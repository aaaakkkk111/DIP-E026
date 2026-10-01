# 2026-09-22 — Always-on Remote Control host setup (infrastructure, not robot work)

Not RL/robot work. Logged here only to keep the session record continuous.
Full write-up lives at `C:\Users\tzeju\claude-pc-setup.md`.

## Goal

Make TZEJUN an always-on Claude Code host drivable from the phone while the
laptop is off, without exposing anything to the internet.

## Outcome

Persistent background session **`claude`** (`efde721a`), Remote Control enabled,
rooted at `C:\Users\tzeju\Desktop\E3180 DIP`.
Link: `https://claude.ai/code/session_014myMPntktGEyyvbjG4RjjV`

## Findings worth keeping

- **tmux does not apply here.** Not installed, cannot run natively on Windows
  (needs a Unix PTY), and WSL is not installed. Claude Code's built-in `--bg`
  sessions are the correct native equivalent and integrate with Remote Control.
- **`--bg` and `--remote-control` combine.** Verified: a session created with both
  gets a `bridgeSessionId` field in `~/.claude/sessions/<pid>.json`. A session
  without Remote Control has no such field — this is the reliable way to check
  whether Remote Control actually registered.
- **Background sessions are genuinely detached.** The session process is a child of
  a daemon-managed `--bg-pty-host`, not of the launching shell.
- **Detach key was documented wrong.** The 19 Sep write-up said `Ctrl+B`, `D`
  (tmux's binding). `claude attach --help` confirms it is **`Ctrl+Z`** (back to
  shell) or **`←`** (agent view). Corrected in the main doc.
- The two sessions from 19 Sep (`e61fa46d`, `d789d96e`) had exited; background
  sessions do not survive reboot or log off.
- Remote Control was **already paired** with the phone on this machine
  (`hasUsedRemoteControl: true`), so no pairing step was needed.
- **Resume preserves the phone link.** Stopping and resuming `efde721a` kept the
  same `bridgeSessionId` (`session_014myMPntktGEyyvbjG4RjjV`), so the
  `claude.ai/code/...` URL is stable across restarts and can be bookmarked.
  `--bg --resume <full-uuid>` reports: *"woke session efde721a with its saved
  options (--remote-control, -n, --model)"* — Remote Control comes back by itself.
- **PowerShell 5.1 gotcha (cost a real bug):** `ConvertFrom-Json` emits a JSON
  array as ONE object, so `@($raw | ConvertFrom-Json)` produces an array
  containing an array. Filters then match the wrapper and property access
  member-enumerates every element at once. Assign to a variable first, then `@()`.

## Follow-up 2026-09-23: why sessions kept dying

**Root cause: Claude Code retires a background session after 8 hours idle.**
Built-in policy, not a crash. In `~/.claude/daemon.log`:

```
[2026-09-20T00:03:33Z] [bg] bg retire d789d96e: settled, idle 8h
[2026-09-20T12:35:27Z] [bg] bg retire 379fc97c: idle-prompt, idle 8h
[2026-09-22T10:24:54Z] [bg] bg retire efde721a: settled, idle 8h
```

This explains every session loss so far, including the two from 19 Sep that were
originally written off as simply "did not survive". Only *idle* time counts;
active work resets it. The `efde721a` case ran 02:11:53 → 10:24:54 (~8h13m).

**Missed on day one.** The line `bg retire 379fc97c: idle-prompt, idle 8h` was
already present in the daemon log read during the initial inspection. It was seen
and not acted on — the focus was on proving the session survives terminal close
(which it does) rather than on what happens when nothing touches it for hours.

No supported override found. `CLAUDE_CODE_IDLE_THRESHOLD_MINUTES` exists in the
binary but there is no evidence it controls this timer, so it was not used.

**Fix (user-approved):** Windows Scheduled Task **"Keep Claude Alive"** — at logon
plus every 6 hours, running `start-claude.ps1` hidden as a Limited user with a
5-minute cap. Six hours beats the 8-hour deadline. Calls the `.ps1` directly, never
the `.cmd`, because the wrapper ends in `pause` and would hang in a hidden window.
Verified both ways: no-op when alive, revives to the same id/UUID/bridge when stopped.

## Environment as checked

Windows 11 Pro 26200 · i7-12700KF · 31.8 GB RAM · C: 589 GB free
Node v24.12.0 · npm 11.6.2 · Python 3.12.9 · Git 2.52.0
Claude Code 2.1.278 — already the latest published version, already authenticated (Pro)

## Changed

- Created session `claude`.
- Added `C:\Users\tzeju\start-claude.ps1` (idempotent — refuses duplicates) and
  `start-claude.cmd`.
- Rewrote `C:\Users\tzeju\claude-pc-setup.md`, merging the 19 Sep findings and
  fixing the detach-key error.

## Deliberately not changed

Firewall (all profiles on), network profile (left Public — the stricter setting;
Remote Control is outbound-only so it costs nothing), power settings (AC sleep and
hibernate already "never"), BIOS, SSH (server not installed — not needed), and no
auto-start service.
