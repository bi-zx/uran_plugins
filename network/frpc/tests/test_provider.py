from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from uran_frpc_plugin.provider import certificate, frpc_config


def test_short_lived_ssh_certificate_and_fixed_loopback_proxy(tmp_path):
    ca, agent = tmp_path / "ca", tmp_path / "agent"
    for path in (ca, agent):
        subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-f", str(path)], check=True)
    service = {"ssh_ca_key": str(ca), "ssh_principal": "uran-maintenance", "ssh_user": "uran-maintenance",
        "server_addr": "frps.example.test", "server_port": 7000,
        "remote_port": 6022, "frps_token": "test-token", "tls_server_name": "frps.example.test",
        "tls_ca_file": "/etc/uran/frps-ca.pem"}
    cert_dir = tmp_path / "certificate"
    cert_dir.mkdir()
    cert = certificate(service, agent.with_suffix(".pub").read_text(), cert_dir, "session-1", 60)
    assert "-cert-v01@openssh.com " in cert
    info = subprocess.run(["ssh-keygen", "-L", "-f", str(cert_dir / "agent-cert.pub")],
                          check=True, stdout=subprocess.PIPE).stdout.decode()
    assert "uran-maintenance" in info and "source-address 127.0.0.1/32" in info
    assert "permit-pty" in info and "permit-port-forwarding" not in info and "permit-agent-forwarding" not in info
    text = frpc_config(service, "session-1")
    assert 'localIP = "127.0.0.1"' in text and "localPort = 2222" in text
    assert text.count("[[proxies]]") == 1 and "transport.tls.enable = true" in text
