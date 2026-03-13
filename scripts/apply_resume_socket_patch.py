#!/usr/bin/env python3
# Copyright 2025 Analog Life LLC
# SPDX-License-Identifier: Apache-2.0
"""
Sync the known-good BG95 resume-socket patch pack into a compatible Zephyr/Golioth
workspace.

This is intentionally conservative:
- it backs up files before overwriting them
- it checks for anchor text in existing targets unless --force is used
- it can dry-run without touching the workspace

It is not a universal patcher for arbitrary upstream versions. It is designed for
compatible workspace layouts that resemble the current RAK5010 tracker environment.
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import hashlib
import shutil
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent


@dataclasses.dataclass(frozen=True)
class SyncItem:
    name: str
    source: Path
    target_rel: Path
    anchors: tuple[str, ...]


def sha256sum(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def read_text_lossy(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def ensure_compatible(target: Path, anchors: tuple[str, ...], force: bool) -> None:
    if force or not target.exists() or not anchors:
        return

    text = read_text_lossy(target)
    missing = [anchor for anchor in anchors if anchor not in text]
    if missing:
        raise RuntimeError(
            f"{target} does not look compatible; missing anchors: {missing}"
        )


def backup_file(target: Path, backup_root: Path) -> None:
    rel = target.drive.replace(":", "") / target.relative_to(target.anchor)
    backup_path = backup_root / rel
    backup_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(target, backup_path)


def sync_item(
    item: SyncItem,
    workspace_root: Path,
    backup_root: Path | None,
    dry_run: bool,
    force: bool,
) -> str:
    target = workspace_root / item.target_rel

    if not item.source.exists():
        raise FileNotFoundError(f"missing source file: {item.source}")

    ensure_compatible(target, item.anchors, force)

    src_hash = sha256sum(item.source)
    if target.exists():
        dst_hash = sha256sum(target)
        if src_hash == dst_hash:
            return f"SKIP  {item.name}: unchanged"
    else:
        dst_hash = None

    if dry_run:
        action = "CREATE" if not target.exists() else "UPDATE"
        return f"DRY   {item.name}: {action} {target}"

    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() and backup_root is not None:
        backup_file(target, backup_root)

    shutil.copy2(item.source, target)
    action = "CREATE" if dst_hash is None else "UPDATE"
    return f"APPLY {item.name}: {action} {target}"


def build_manifest(app_name: str) -> dict[str, list[SyncItem]]:
    app_root = REPO_ROOT / "files" / "app" / app_name
    return {
        "sdk": [
            SyncItem(
                name="sdk-coap-c",
                source=REPO_ROOT / "files" / "golioth-sdk" / "coap_client_zephyr.c",
                target_rel=Path("modules/lib/golioth-firmware-sdk/src/coap_client_zephyr.c"),
                anchors=("static int golioth_connect_host_port(", "struct golioth_client _golioth_client;"),
            ),
            SyncItem(
                name="sdk-coap-h",
                source=REPO_ROOT / "files" / "golioth-sdk" / "coap_client_zephyr.h",
                target_rel=Path("modules/lib/golioth-firmware-sdk/src/coap_client_zephyr.h"),
                anchors=("struct golioth_client", "int sock;"),
            ),
            SyncItem(
                name="sdk-kconfig",
                source=REPO_ROOT / "files" / "golioth-sdk" / "port-zephyr-Kconfig",
                target_rel=Path("modules/lib/golioth-firmware-sdk/port/zephyr/Kconfig"),
                anchors=("config GOLIOTH_COAP_CLIENT_RX_TIMEOUT_SEC", "menuconfig GOLIOTH_FIRMWARE_SDK"),
            ),
        ],
        "driver": [
            SyncItem(
                name="driver-bg9x-c",
                source=REPO_ROOT / "files" / "zephyr-driver" / "quectel-bg9x.c",
                target_rel=Path("zephyr/drivers/modem/quectel-bg9x.c"),
                anchors=("int bg9x_psm_set_interval(", "int bg9x_cell_info_get("),
            ),
            SyncItem(
                name="driver-bg9x-h",
                source=REPO_ROOT / "files" / "zephyr-driver" / "quectel-bg9x.h",
                target_rel=Path("zephyr/drivers/modem/quectel-bg9x.h"),
                anchors=("struct modem_data {", "bool runtime_wake_pending;"),
            ),
            SyncItem(
                name="driver-bg9x-public-h",
                source=REPO_ROOT / "files" / "zephyr-include" / "quectel_bg9x.h",
                target_rel=Path("zephyr/include/zephyr/drivers/modem/quectel_bg9x.h"),
                anchors=("struct bg9x_runtime_stats", "int bg9x_psm_set_interval"),
            ),
            SyncItem(
                name="driver-kconfig",
                source=REPO_ROOT / "files" / "zephyr-driver" / "Kconfig.quectel-bg9x",
                target_rel=Path("zephyr/drivers/modem/Kconfig.quectel-bg9x"),
                anchors=("config MODEM_QUECTEL_BG9X",),
            ),
        ],
        "app": [
            SyncItem(
                name="app-main",
                source=app_root / "src" / "main.c",
                target_rel=Path(f"apps/{app_name}/src/main.c"),
                anchors=("rak5010 tracker start", "SHELL_CMD_REGISTER(resume_socket"),
            ),
            SyncItem(
                name="app-debug-conf",
                source=app_root / "boards" / "rak5010_nrf52840_debugcycle.conf",
                target_rel=Path(f"apps/{app_name}/boards/rak5010_nrf52840_debugcycle.conf"),
                anchors=("CONFIG_GOLIOTH_USE_CONNECTION_ID",),
            ),
            SyncItem(
                name="app-offload-conf",
                source=app_root / "boards" / "rak5010_nrf52840_offload.conf",
                target_rel=Path(f"apps/{app_name}/boards/rak5010_nrf52840_offload.conf"),
                anchors=("CONFIG_GOLIOTH_COAP_CLIENT_SKIP_INITIAL_EMPTY",),
            ),
            SyncItem(
                name="app-prod-conf",
                source=app_root / "boards" / "rak5010_nrf52840.conf",
                target_rel=Path(f"apps/{app_name}/boards/rak5010_nrf52840.conf"),
                anchors=("CONFIG_GOLIOTH_COAP_CLIENT_SKIP_INITIAL_EMPTY",),
            ),
            SyncItem(
                name="app-overlay",
                source=app_root / "boards" / "rak5010_nrf52840_offload.overlay",
                target_rel=Path(f"apps/{app_name}/boards/rak5010_nrf52840_offload.overlay"),
                anchors=("zephyr,console", "uart1"),
            ),
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Apply the RAK5010 BG95 resume-socket patch pack to a compatible workspace."
    )
    parser.add_argument(
        "--workspace-root",
        default=r"C:\path\to\golioth-workspace",
        help="Target workspace root to patch.",
    )
    parser.add_argument(
        "--app-name",
        default="rak5010_tracker",
        help="Application directory name under apps/ in the target workspace.",
    )
    parser.add_argument(
        "--with-app",
        action="store_true",
        help="Also sync the included reference application files. By default only SDK and driver files are applied.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Report actions without changing files.")
    parser.add_argument("--force", action="store_true", help="Skip compatibility anchor checks.")
    parser.add_argument("--no-sdk", action="store_true", help="Do not sync SDK files.")
    parser.add_argument("--no-driver", action="store_true", help="Do not sync driver files.")
    parser.add_argument("--no-app", action="store_true", help="Do not sync app files, even if --with-app is set.")
    parser.add_argument(
        "--backup-root",
        help="Optional backup directory. Defaults to <workspace>/.rak5010_patch_backups/<timestamp>.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    workspace_root = Path(args.workspace_root).resolve()
    if not workspace_root.exists():
        print(f"workspace root not found: {workspace_root}", file=sys.stderr)
        return 2

    manifest = build_manifest(args.app_name)
    enabled_groups: list[str] = []
    if not args.no_sdk:
        enabled_groups.append("sdk")
    if not args.no_driver:
        enabled_groups.append("driver")
    if args.with_app and not args.no_app:
        enabled_groups.append("app")

    if not enabled_groups:
        print("nothing to do: all groups disabled", file=sys.stderr)
        return 2

    backup_root: Path | None
    if args.dry_run:
        backup_root = None
    else:
        if args.backup_root:
            backup_root = Path(args.backup_root).resolve()
        else:
            stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
            backup_root = workspace_root / ".rak5010_patch_backups" / stamp
        backup_root.mkdir(parents=True, exist_ok=True)

    messages: list[str] = []
    try:
        for group in enabled_groups:
            messages.append(f"[{group}]")
            for item in manifest[group]:
                messages.append(
                    sync_item(
                        item=item,
                        workspace_root=workspace_root,
                        backup_root=backup_root,
                        dry_run=args.dry_run,
                        force=args.force,
                    )
                )
    except Exception as exc:  # pragma: no cover - explicit failure path
        print(f"patch failed: {exc}", file=sys.stderr)
        return 1

    for msg in messages:
        print(msg)

    if not args.dry_run and backup_root is not None:
        print(f"backups: {backup_root}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
