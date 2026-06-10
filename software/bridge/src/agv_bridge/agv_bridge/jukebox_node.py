"""Jukebox node — background music for REMOTE_CONTROL driving.

Mirrors the same /agv/cmd_vel stream the GUI pushes at 20 Hz while in
REMOTE_CONTROL: plays a local audio file the instant a non-zero (v, ω)
arrives, pauses the moment the stream goes idle (zero command or the stream
itself goes stale — e.g. the operator leaves REMOTE_CONTROL or disconnects),
and resumes from the same playback position on the next motion command.

Drop a song at the path given by the ``song_path`` parameter (default
``~/agv_song.mp3``); any format SDL_mixer can decode works (mp3, ogg, wav...).
"""

from __future__ import annotations

import os
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import pygame


class JukeboxNode(Node):
    def __init__(self) -> None:
        super().__init__('jukebox_node')

        self.declare_parameter('song_path', '~/agv_song.mp3')
        self.declare_parameter('cmd_vel_topic', '/agv/cmd_vel')
        self.declare_parameter('deadband', 1e-3)
        self.declare_parameter('stale_timeout_s', 0.5)

        g = self.get_parameter
        self._path = os.path.expanduser(str(g('song_path').value))
        self._deadband = float(g('deadband').value)
        self._stale_s = float(g('stale_timeout_s').value)

        self._ready = False
        self._started = False   # play() called at least once (vs. paused)
        self._playing = False
        self._last_moving_ts = 0.0

        try:
            pygame.mixer.init()
            pygame.mixer.music.load(self._path)
            self._ready = True
            self.get_logger().info(f'jukebox_node: loaded {self._path}')
        except Exception as exc:
            self.get_logger().warn(
                f'jukebox_node: no playable song at {self._path} ({exc}); staying silent')

        self.create_subscription(Twist, str(g('cmd_vel_topic').value), self._on_cmd_vel, 10)
        self.create_timer(0.2, self._check_stale)

    def _on_cmd_vel(self, msg: Twist) -> None:
        moving = abs(msg.linear.x) > self._deadband or abs(msg.angular.z) > self._deadband
        if moving:
            self._last_moving_ts = time.monotonic()
            self._play()
        else:
            self._pause()

    def _check_stale(self) -> None:
        if self._playing and (time.monotonic() - self._last_moving_ts) > self._stale_s:
            self._pause()

    def _play(self) -> None:
        if not self._ready or self._playing:
            return
        if self._started:
            pygame.mixer.music.unpause()
        else:
            pygame.mixer.music.play(loops=-1)
            self._started = True
        self._playing = True

    def _pause(self) -> None:
        if not self._ready or not self._playing:
            return
        pygame.mixer.music.pause()
        self._playing = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JukeboxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
