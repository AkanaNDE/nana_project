#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
import re
import time
import subprocess
import matplotlib
matplotlib.use('Agg')  # ← ไม่ใช้ GUI backend เพื่อกัน not responding
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

class ControlPID(Node):
    def __init__(self):
        super().__init__('control_pid')

        # PID gains
        self.Kp = 0.002
        self.Ki = 0.0001
        self.Kd = 0.001

        self.prev_error  = 0.0
        self.integral    = 0.0
        self.prev_time   = time.monotonic()

        self.last_ticks  = None
        self.left_accum  = 0.0
        self.right_accum = 0.0

        # ── Log data ──
        self.log_time       = []
        self.log_error      = []
        self.log_correction = []
        self.log_left       = []
        self.log_right      = []
        self.log_p_term     = []
        self.log_i_term     = []
        self.log_d_term     = []

        self.start_time = time.monotonic()
        self.finished   = False

        # Subscriptions
        self.create_subscription(String,  '/encoder_ticks',       self.encoder_cb,  10)
        self.create_subscription(Bool,    '/finish_pid',          self.finish_cb,   10)
        self.create_subscription(Float32, '/current_distance_cm', self.distance_cb, 10)

        # Publisher
        self.correction_pub = self.create_publisher(Float32, '/pid_correction', 10)

        self.get_logger().info("ControlPID Node Started (E3, E4 only)")

    # ──────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────

    def distance_cb(self, msg):
        pass  # reserved for future use

    def encoder_cb(self, msg):
        if self.finished:
            return
        try:
            vals = [int(x) for x in re.findall(r'-?\d+', msg.data)]
            if len(vals) < 4:
                return

            if self.last_ticks is None:
                self.last_ticks = vals
                return

            d3 = vals[2] - self.last_ticks[2]
            d4 = vals[3] - self.last_ticks[3]

            if d3 >  30000: d3 -= 65536
            if d3 < -30000: d3 += 65536
            if d4 >  30000: d4 -= 65536
            if d4 < -30000: d4 += 65536

            self.last_ticks = vals

            self.left_accum  += abs(d3)
            self.right_accum += abs(d4)

            error = self.left_accum - self.right_accum

            now = time.monotonic()
            dt  = now - self.prev_time
            if dt <= 0:
                return
            self.prev_time = now

            self.integral  += error * dt
            derivative      = (error - self.prev_error) / dt
            self.prev_error = error

            self.integral = max(-500.0, min(500.0, self.integral))

            p_term = self.Kp * error
            i_term = self.Ki * self.integral
            d_term = self.Kd * derivative

            correction = p_term + i_term + d_term
            correction = max(-0.3, min(0.3, correction))

            out      = Float32()
            out.data = float(correction)
            self.correction_pub.publish(out)

            elapsed = now - self.start_time
            self.log_time.append(elapsed)
            self.log_error.append(error)
            self.log_correction.append(correction)
            self.log_left.append(self.left_accum)
            self.log_right.append(self.right_accum)
            self.log_p_term.append(p_term)
            self.log_i_term.append(i_term)
            self.log_d_term.append(d_term)

            self.get_logger().info(
                f"E3={vals[2]} E4={vals[3]} | "
                f"L={self.left_accum:.0f} R={self.right_accum:.0f} | "
                f"err={error:.1f} corr={correction:.4f}",
                throttle_duration_sec=1.0
            )

        except Exception as e:
            self.get_logger().error(f"PID error: {e}")

    def finish_cb(self, msg):
        if not msg.data or self.finished:
            return
        self.finished = True
        self.get_logger().info("/finish_pid received → generating graph...")
        self.plot_graph()  # ← เรียกตรงๆ ใน main thread ไม่ใช้ thread แยก

    # ──────────────────────────────────────────
    # Plot
    # ──────────────────────────────────────────

    def plot_graph(self):
        if not self.log_time:
            self.get_logger().warn("No data to plot.")
            return

        t   = np.array(self.log_time)
        err = np.array(self.log_error)
        cor = np.array(self.log_correction)
        lft = np.array(self.log_left)
        rgt = np.array(self.log_right)
        p   = np.array(self.log_p_term)
        i_  = np.array(self.log_i_term)
        d   = np.array(self.log_d_term)

        # ── สี ──
        C_ERR  = '#e74c3c'
        C_COR  = '#2980b9'
        C_LEFT = '#27ae60'
        C_RIGT = '#e67e22'
        C_P    = '#8e44ad'
        C_I    = '#16a085'
        C_D    = '#d35400'
        C_REF  = '#95a5a6'

        total_duration = float(t[-1] - t[0]) if len(t) > 1 else 1.0
        sample_rate    = len(t) / total_duration if total_duration > 0 else 0
        max_abs_err    = float(np.max(np.abs(err)))
        mean_abs_err   = float(np.mean(np.abs(err)))
        final_err      = float(err[-1])
        max_cor        = float(np.max(np.abs(cor)))

        # ── Layout ──
        fig = plt.figure(figsize=(15, 18))
        fig.patch.set_facecolor('#1a1a2e')

        gs = gridspec.GridSpec(
            5, 2,
            figure=fig,
            hspace=0.55,
            wspace=0.35,
            left=0.07, right=0.97,
            top=0.92,  bottom=0.05
        )

        ax1 = fig.add_subplot(gs[0, :])
        ax2 = fig.add_subplot(gs[1, :])
        ax3 = fig.add_subplot(gs[2, :])
        ax4 = fig.add_subplot(gs[3, 0])
        ax5 = fig.add_subplot(gs[3, 1])
        ax6 = fig.add_subplot(gs[4, :])

        for ax in [ax1, ax2, ax3, ax4, ax5, ax6]:
            ax.set_facecolor('#16213e')
            ax.tick_params(colors='white', labelsize=9)
            ax.xaxis.label.set_color('white')
            ax.yaxis.label.set_color('white')
            ax.title.set_color('white')
            for spine in ax.spines.values():
                spine.set_edgecolor('#4a4a6a')

        def grid_style(ax):
            ax.grid(True, color='#2a2a4a', linewidth=0.7, linestyle='--', alpha=0.8)

        # ────────────────────────────────────────
        # กราฟ 1: Tick Error
        # ────────────────────────────────────────
        ax1.plot(t, err, color=C_ERR, linewidth=1.6,
                 label='Error = Left − Right (ticks)', zorder=3)
        ax1.axhline(0, color=C_REF, linewidth=1.0, linestyle='--',
                    label='Reference (0)', zorder=2)
        ax1.fill_between(t, err, 0, where=(err > 0),
                         alpha=0.15, color=C_ERR, label='Left faster')
        ax1.fill_between(t, err, 0, where=(err < 0),
                         alpha=0.15, color=C_LEFT, label='Right faster')
        ax1.set_title("① Encoder Tick Error  (Left − Right Accumulated)",
                      fontsize=11, fontweight='bold', pad=8)
        ax1.set_ylabel("Error (ticks)", fontsize=9)
        ax1.set_xlabel("Time (s)", fontsize=9)
        ax1.legend(loc='upper right', fontsize=8,
                   facecolor='#1a1a2e', labelcolor='white', framealpha=0.8)
        ax1.annotate(f"Max |err| = {max_abs_err:.1f}  |  Mean |err| = {mean_abs_err:.1f}  |  Final err = {final_err:.1f}",
                     xy=(0.01, 0.92), xycoords='axes fraction', fontsize=8, color='#f1c40f',
                     bbox=dict(boxstyle='round,pad=0.3', facecolor='#1a1a2e',
                               edgecolor='#f1c40f', alpha=0.8))
        grid_style(ax1)

        # ────────────────────────────────────────
        # กราฟ 2: PID Correction Output
        # ────────────────────────────────────────
        ax2.plot(t, cor, color=C_COR, linewidth=1.6,
                 label='PID Correction → angular.z', zorder=3)
        ax2.axhline(0,    color=C_REF,     linewidth=1.0, linestyle='--', zorder=2)
        ax2.axhline( 0.3, color='#e74c3c', linewidth=0.9, linestyle=':', label='+0.3 limit')
        ax2.axhline(-0.3, color='#e74c3c', linewidth=0.9, linestyle=':', label='−0.3 limit')
        ax2.fill_between(t, cor,  0.3, where=(cor >=  0.3),
                         alpha=0.25, color='#e74c3c')
        ax2.fill_between(t, cor, -0.3, where=(cor <= -0.3),
                         alpha=0.25, color='#e74c3c')
        ax2.set_title("② PID Controller Output  (angular.z correction)",
                      fontsize=11, fontweight='bold', pad=8)
        ax2.set_ylabel("Correction", fontsize=9)
        ax2.set_xlabel("Time (s)", fontsize=9)
        ax2.set_ylim(-0.45, 0.45)
        ax2.legend(loc='upper right', fontsize=8,
                   facecolor='#1a1a2e', labelcolor='white', framealpha=0.8)
        ax2.annotate(f"Max |correction| = {max_cor:.4f}",
                     xy=(0.01, 0.92), xycoords='axes fraction', fontsize=8, color='#f1c40f',
                     bbox=dict(boxstyle='round,pad=0.3', facecolor='#1a1a2e',
                               edgecolor='#f1c40f', alpha=0.8))
        grid_style(ax2)

        # ────────────────────────────────────────
        # กราฟ 3: Left vs Right Accumulated Ticks
        # ────────────────────────────────────────
        ax3.plot(t, lft, color=C_LEFT, linewidth=1.8,
                 label='Left  E3 accumulated (ticks)')
        ax3.plot(t, rgt, color=C_RIGT, linewidth=1.8, linestyle='--',
                 label='Right E4 accumulated (ticks)')
        ax3.fill_between(t, lft, rgt, alpha=0.12, color='white',
                         label='Imbalance area')
        ax3.set_title("③ Left vs Right Accumulated Encoder Ticks",
                      fontsize=11, fontweight='bold', pad=8)
        ax3.set_ylabel("Accumulated Ticks", fontsize=9)
        ax3.set_xlabel("Time (s)", fontsize=9)
        ax3.legend(loc='upper left', fontsize=8,
                   facecolor='#1a1a2e', labelcolor='white', framealpha=0.8)
        grid_style(ax3)

        # ────────────────────────────────────────
        # กราฟ 4: P / I / D Terms Breakdown
        # ────────────────────────────────────────
        ax4.plot(t, p,  color=C_P, linewidth=1.4, label=f'P  (Kp={self.Kp})')
        ax4.plot(t, i_, color=C_I, linewidth=1.4, linestyle='--',
                 label=f'I  (Ki={self.Ki})')
        ax4.plot(t, d,  color=C_D, linewidth=1.4, linestyle=':',
                 label=f'D  (Kd={self.Kd})')
        ax4.axhline(0, color=C_REF, linewidth=0.8, linestyle='--')
        ax4.set_title("④ PID Term Breakdown", fontsize=10, fontweight='bold', pad=6)
        ax4.set_ylabel("Term Value", fontsize=9)
        ax4.set_xlabel("Time (s)", fontsize=9)
        ax4.legend(loc='upper right', fontsize=7.5,
                   facecolor='#1a1a2e', labelcolor='white', framealpha=0.8)
        grid_style(ax4)

        # ────────────────────────────────────────
        # กราฟ 5: Error Rate of Change
        # ────────────────────────────────────────
        if len(err) > 1:
            dt_arr   = np.diff(t)
            dt_arr   = np.where(dt_arr <= 0, 1e-6, dt_arr)
            err_rate = np.diff(err) / dt_arr
            ax5.plot(t[1:], err_rate, color='#f39c12', linewidth=1.2,
                     label='d(error)/dt  (tick/s)')
            ax5.axhline(0, color=C_REF, linewidth=0.8, linestyle='--')
            ax5.set_title("⑤ Error Rate of Change  d(error)/dt",
                          fontsize=10, fontweight='bold', pad=6)
            ax5.set_ylabel("Tick/s", fontsize=9)
            ax5.set_xlabel("Time (s)", fontsize=9)
            ax5.legend(loc='upper right', fontsize=7.5,
                       facecolor='#1a1a2e', labelcolor='white', framealpha=0.8)
        grid_style(ax5)

        # ────────────────────────────────────────
        # กราฟ 6: Cumulative Imbalance %
        # ────────────────────────────────────────
        total_ticks = (lft + rgt) / 2.0
        with np.errstate(divide='ignore', invalid='ignore'):
            imbalance_pct = np.where(
                total_ticks > 0, (err / total_ticks) * 100.0, 0.0)

        ax6.plot(t, imbalance_pct, color='#1abc9c', linewidth=1.5,
                 label='Imbalance %  (error / avg_ticks × 100)')
        ax6.axhline(0,  color=C_REF,     linewidth=1.0, linestyle='--')
        ax6.axhline( 5, color='#e74c3c', linewidth=0.8, linestyle=':',
                    label='±5% threshold')
        ax6.axhline(-5, color='#e74c3c', linewidth=0.8, linestyle=':')
        ax6.fill_between(t, imbalance_pct, 0,
                         where=(np.abs(imbalance_pct) > 5),
                         alpha=0.2, color='#e74c3c', label='Out of ±5% zone')
        ax6.set_title("⑥ Wheel Imbalance Percentage",
                      fontsize=10, fontweight='bold', pad=6)
        ax6.set_ylabel("Imbalance (%)", fontsize=9)
        ax6.set_xlabel("Time (s)", fontsize=9)
        ax6.legend(loc='upper right', fontsize=7.5,
                   facecolor='#1a1a2e', labelcolor='white', framealpha=0.8)
        grid_style(ax6)

        # ────────────────────────────────────────
        # Title + Statistics
        # ────────────────────────────────────────
        stats_text = (
            f"Duration: {total_duration:.1f} s     "
            f"Samples: {len(t)}     "
            f"Sample rate: {sample_rate:.1f} Hz     "
            f"Mean |error|: {mean_abs_err:.1f} ticks     "
            f"Final error: {final_err:.1f} ticks     "
            f"Max |correction|: {max_cor:.4f}"
        )
        fig.suptitle(
            "PID Straight-Line Control — Full Analysis Report",
            fontsize=14, fontweight='bold', color='white', y=0.975
        )
        fig.text(
            0.5, 0.955, stats_text,
            ha='center', va='center', fontsize=8.5, color='#f1c40f',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#16213e',
                      edgecolor='#f1c40f', alpha=0.9)
        )

        # ── Save ──
        save_path = '/tmp/pid_report.png'
        plt.savefig(save_path, dpi=150, bbox_inches='tight',
                    facecolor=fig.get_facecolor())
        plt.close(fig)
        self.get_logger().info(f"Graph saved → {save_path}")

        # ── เปิดภาพด้วย system viewer ──
        for viewer in ['eog', 'xdg-open', 'display', 'feh']:
            try:
                subprocess.Popen([viewer, save_path],
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                self.get_logger().info(f"Opened with: {viewer}")
                break
            except FileNotFoundError:
                continue

    def reset(self):
        self.left_accum  = 0.0
        self.right_accum = 0.0
        self.integral    = 0.0
        self.prev_error  = 0.0
        self.last_ticks  = None
        self.prev_time   = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = ControlPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()