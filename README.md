# zephyr-bg95-socket-resume

Patch pack for **Zephyr + Golioth + BG95** workspaces that need BG95-native PSM support and lower-overhead resume behavior after sleep.

What this adds:
- BG95-native PSM interval control and runtime wake handling
- BG95 modem-socket presence check and recreate via `QISTATE` / `QIOPEN`
- DTLS CID-friendly resume path
- DNS cache-first reconnect
- initial empty CoAP probe skip
- optional reference app files used to validate the path on RAK5010

## What this is

This repo is a **patch pack**, not a standalone driver release.

It includes:
- `files/zephyr-driver/` — BG95 driver changes
- `files/zephyr-include/` — public BG95 header
- `files/golioth-sdk/` — Golioth Zephyr transport changes
- `files/app/` — optional reference app integration
- `scripts/apply_resume_socket_patch.py` — workspace patcher

## What it improves

Reference measured results from the working BG95 tracker environment:

- cold connect only: about `1166 B`
- cold connect + `/tracker`: about `1637 B`
- resumed `/tracker` send: about `361 tx / 110 rx / 471 total`
- resumed tiny 1-byte probe: about `121 tx / 110 rx / 231 total`

The important part is the resumed path: the large reconnect bucket is avoided when the socket-resume path works.

## Current behavior

- automatic modem socket reopen: **yes**
- automatic full reconnect fallback after resume failure: **not included yet**
- DNS cache survives reconnects in the same boot: **yes**
- DNS cache survives reboot: **no**

Observed boundary in the reference environment:
- `45-57m` resume windows worked
- around the `~60m+` class, resume became less reliable and should be validated in your own environment

## Apply the patch

By default, the patcher applies **SDK + driver only**.

Dry-run:

```powershell
python scripts\apply_resume_socket_patch.py --dry-run --workspace-root "C:\path\to\golioth-workspace"
```

Apply:

```powershell
python scripts\apply_resume_socket_patch.py --workspace-root "C:\path\to\golioth-workspace"
```

Also copy the included reference app files:

```powershell
python scripts\apply_resume_socket_patch.py --workspace-root "C:\path\to\golioth-workspace" --with-app
```

Useful flags:

```powershell
python scripts\apply_resume_socket_patch.py --no-sdk
python scripts\apply_resume_socket_patch.py --no-driver
python scripts\apply_resume_socket_patch.py --with-app --app-name my_app
python scripts\apply_resume_socket_patch.py --force
```

The script is conservative:
- backs up overwritten files
- checks compatibility anchors unless `--force` is used
- is intended for compatible workspace layouts, not arbitrary upstream versions

## Important caveat

If you only copy the driver, you do **not** get the full measured result from the reference environment.

The proven low-overhead path depends on:
- driver changes
- Golioth transport changes
- app policy about when to reuse vs reconnect

So treat this repo as:
- **driver + SDK patch pack**
- with an **optional reference app**

not as a single drop-in source file.
