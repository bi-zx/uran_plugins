"""Supervisor-only JSONL entry point. No remote command/configuration API."""
import ctypes
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def certificate(service, public_key, directory, session_id, ttl):
    if "\n" in public_key.strip() or "\x00" in public_key:
        raise ValueError("one SSH public key is required")
    path = Path(directory) / "agent.pub"
    path.write_text(public_key.strip() + "\n")
    principal = service["ssh_principal"]
    if not principal or any(char not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for char in principal):
        raise ValueError("invalid local SSH principal")
    if service.get("ssh_user") == "root":
        raise ValueError("remote maintenance requires a non-root SSH account")
    subprocess.run(["ssh-keygen", "-l", "-f", str(path)], check=True,
                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
    subprocess.run(["ssh-keygen", "-q", "-s", service["ssh_ca_key"], "-I", session_id,
        "-n", principal, "-V", "-30s:+%ds" % ttl,
        # sshd sees FRPC's loopback source, not the original cloud address.
        # Any cloud-IP filtering must be applied on FRPS's exposed TCP port.
        "-O", "clear", "-O", "permit-pty", "-O", "source-address=127.0.0.1/32", str(path)],
        check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
    return path.with_name("agent-cert.pub").read_text().strip()


def frpc_config(service, session_id):
    def quote(value):
        return json.dumps(value, ensure_ascii=False)
    port = int(service["remote_port"])
    if not 1 <= port <= 65535:
        raise ValueError("invalid local FRPS allocation")
    local_port = int(service.get("local_ssh_port", 2222))
    if not 1024 <= local_port <= 65535:
        raise ValueError("maintenance must use a dedicated unprivileged loopback SSH port")
    # Exactly one TCP proxy to loopback SSH. The agent cannot select local
    # targets, FRPS servers, remote ports, auth tokens or TLS configuration.
    return '\n'.join([
        'serverAddr = ' + quote(service["server_addr"]),
        'serverPort = ' + str(int(service["server_port"])),
        'auth.method = "token"', 'auth.token = ' + quote(service["frps_token"]),
        'transport.tls.enable = true',
        'transport.tls.serverName = ' + quote(service["tls_server_name"]),
        'transport.tls.trustedCaFile = ' + quote(service["tls_ca_file"]),
        '[[proxies]]', 'name = ' + quote("uran-" + session_id), 'type = "tcp"',
        'localIP = "127.0.0.1"', 'localPort = ' + str(local_port), 'remotePort = ' + str(port), ''])


def main():
    os.umask(0o077)
    request = json.loads(sys.stdin.buffer.readline(65537))
    parent = request["parent_pid"]
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) or os.getppid() != parent:
        return 1
    ttl = min(900, int(request["expires_at"] - time.time()))
    if ttl <= 0:
        raise ValueError("maintenance approval expired")
    running = [True]
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: running.__setitem__(0, False))
    with tempfile.TemporaryDirectory(prefix="uran-frpc-") as directory:
        service = request["service"]
        cert = certificate(service, request["ssh_public_key"], directory, request["session_id"], ttl)
        config = Path(directory) / "frpc.toml"
        config.write_text(frpc_config(service, request["session_id"]))
        binary = service["frpc_binary"]
        if not os.path.isabs(binary):
            raise ValueError("FRPC executable must be provisioned locally")
        process = subprocess.Popen([binary, "-c", str(config)], stdin=subprocess.DEVNULL,
                                   stdout=sys.stderr, stderr=sys.stderr)
        try:
            time.sleep(0.2)
            if process.poll() is not None:
                raise ValueError("FRPC process failed to start")
            print(json.dumps({"ok": True, "connection": {"state": "starting",
                "host": service["server_addr"], "port": service["remote_port"],
                "user": service["ssh_user"], "ssh_certificate": cert}}), flush=True)
            deadline = time.monotonic() + ttl
            while (running[0] and time.monotonic() < deadline and
                   time.time() < request["expires_at"] and process.poll() is None):
                time.sleep(0.1)
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception:
        print('{"ok":false,"error":"maintenance provider failed"}', flush=True)
        raise SystemExit(1)
