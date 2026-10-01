"""TesseractViewer lifecycle: close() releases the server, the loop and the port (gh-150)."""

import socket

from tesseract_robotics.viewer import TesseractViewer

# Seconds to wait for the server socket to accept a TCP connection after start.
CONNECT_TIMEOUT_S = 5.0


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_close_closes_loop_and_releases_port():
    port = _free_port()
    viewer = TesseractViewer(server_address=("127.0.0.1", port))
    viewer.start_serve_background()
    socket.create_connection(("127.0.0.1", port), timeout=CONNECT_TIMEOUT_S).close()

    viewer.close()

    assert not viewer.loop_thread.is_alive()
    assert viewer.loop.is_closed()
    # Port is free again: binding without SO_REUSEADDR succeeds only if the listener is gone.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", port))


def test_close_without_start():
    viewer = TesseractViewer(server_address=("127.0.0.1", _free_port()))
    viewer.close()
    assert viewer.loop.is_closed()


def test_close_is_idempotent():
    viewer = TesseractViewer(server_address=("127.0.0.1", _free_port()))
    viewer.close()
    viewer.close()


def test_serve_forever_starts_and_closes_on_keyboard_interrupt(monkeypatch):
    port = _free_port()
    viewer = TesseractViewer(server_address=("127.0.0.1", port))

    def interrupt(_seconds):
        socket.create_connection(("127.0.0.1", port), timeout=CONNECT_TIMEOUT_S).close()
        raise KeyboardInterrupt

    monkeypatch.setattr("tesseract_robotics.viewer.tesseract_viewer.time.sleep", interrupt)
    viewer.serve_forever()
    assert viewer.loop.is_closed()


def test_close_with_client_holding_connection_open():
    """An idle keep-alive client (a browser tab) must not block close()."""
    port = _free_port()
    viewer = TesseractViewer(server_address=("127.0.0.1", port))
    viewer.start_serve_background()
    with socket.create_connection(("127.0.0.1", port), timeout=CONNECT_TIMEOUT_S) as client:
        client.sendall(b"GET / HTTP/1.1\r\nHost: localhost\r\nConnection: keep-alive\r\n\r\n")
        client.recv(1)
        viewer.close()
        assert viewer.loop.is_closed()


def test_start_returns_once_listening():
    """start_serve_background() returns with the port accepting; no client-side retry."""
    port = _free_port()
    viewer = TesseractViewer(server_address=("127.0.0.1", port))
    viewer.start_serve_background()
    try:
        socket.create_connection(("127.0.0.1", port), timeout=CONNECT_TIMEOUT_S).close()
    finally:
        viewer.close()
