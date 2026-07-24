# TMuLE sim launchers

[TMuLE](https://github.com/marc-hanheide/TMuLE) (the TMux Launch Engine) brings up
a whole multi-process system in a single `tmux` session from one YAML file — one
window per sub-system, one pane per command. These configs replace the manual
copy-paste in `~/.scripts/sowbot_launch.script`.

## Scenarios

| Scenario | File | What it starts |
| --- | --- | --- |
| `row_follow_sim` | [`row_follow_sim.yaml`](row_follow_sim.yaml) | Nav stack (`./manage.py --sim`) + Gazebo (`sowbot_sim.launch.py`) + crop-row CV node (`crop_row_nav.launch.py`) |

All three windows share one Docker container (`sowbot_runtime`). `nav_stack`
creates it; `gazebo` and `crop_row` `docker exec` into it and self-gate until the
container — and the generated world — are ready (helpers in [`lib.sh`](lib.sh)).

## Install (once)

`tmule` needs `tmux`. On Ubuntu 24.04 (PEP-668, "externally managed" Python), the
cleanest install is via `pipx`:

```bash
sudo apt install -y tmux pipx
pipx install tmule
pipx ensurepath          # then open a new shell so `tmule` is on PATH
```

Alternatively, in a virtualenv:

```bash
sudo apt install -y tmux
python3 -m venv ~/.venvs/tmule && ~/.venvs/tmule/bin/pip install tmule
# add ~/.venvs/tmule/bin to PATH, or call ~/.venvs/tmule/bin/tmule directly
```

## Usage

From the repo root:

```bash
# Launch the full row-following sim
tmule -c tmule/row_follow_sim.yaml launch

# Watch it / interact with the panes
tmux attach -t row_follow_sim      # Ctrl-b w = switch window, Ctrl-b d = detach

# Shut it all down (Ctrl-C each pane; manage.py stops the container)
tmule -c tmule/row_follow_sim.yaml stop

# Nuke the session entirely
tmule -c tmule/row_follow_sim.yaml terminate
```

`launch` returns as soon as the panes are started and leaves the tmux session
detached, so attach separately to watch it (and to type the `nav_stack` sudo
password).

### Notes

- **Sudo prompt:** the `nav_stack` pane runs `./manage.py --sim`, which may run
  `sudo python3 fixusb.py`. Attach and type the password in that pane (or set up
  passwordless sudo). Until the container is up, the `gazebo`/`crop_row` panes
  just wait.
- **Timeouts:** the waits are generous (10 min) to cover the sudo prompt and
  first-run world generation. Override with `FF_WAIT_CONTAINER_TIMEOUT` /
  `FF_WAIT_WORLD_TIMEOUT` (seconds) in the environment.
- **Restart just one part:** `tmule -c tmule/row_follow_sim.yaml relaunch -w gazebo`
  (or `stop`/`launch` with `-w <window>`).

## tmux cheat sheet

TMuLE is just a driver for `tmux` — once launched, everything lives in one
detached tmux session named after the scenario (`row_follow_sim`). You don't need
tmux running in the foreground: detach and the stack keeps going.

The session has one window per sub-system, plus a stray `0: bash` window that
tmux creates with every new session (harmless — ignore it):

```
0: bash   1: nav_stack   2: gazebo   3: crop_row
```

### From the shell

```bash
tmux ls                                # list sessions
tmux attach -t row_follow_sim          # attach
tmux kill-session -t row_follow_sim    # nuke the session (tmule terminate does this too)
```

### Inside the session

Every shortcut starts with the **prefix**, `Ctrl-b` — press and release it, then
the next key.

| Keys | Does |
| --- | --- |
| `Ctrl-b` `d` | **Detach** — leaves everything running in the background |
| `Ctrl-b` `w` | Interactive window picker (easiest way to move around) |
| `Ctrl-b` `1` / `2` / `3` | Jump straight to `nav_stack` / `gazebo` / `crop_row` |
| `Ctrl-b` `n` / `p` | Next / previous window |
| `Ctrl-b` `[` | **Scroll back** through output — arrows/PgUp/PgDn, `q` to exit |
| `Ctrl-b` `z` | Zoom the current pane fullscreen (press again to unzoom) |
| `Ctrl-b` `?` | List every binding |

Scrolling back (`Ctrl-b` `[`) is the one you'll use most — it's how you read ROS
log output that has already scrolled past. To scroll with the mouse wheel
instead, add `set -g mouse on` to `~/.tmux.conf`.

`Ctrl-c` in a pane stops just that process, which is what `tmule stop` sends.
Note that detaching is **not** stopping: use
`tmule -c tmule/<scenario>.yaml stop` to shut the stack down.

## Adding a scenario

Copy `row_follow_sim.yaml` to `tmule/<name>.yaml`, set `session: <name>`, and
edit the windows. `tmule -c tmule/<name>.yaml launch` and `tmux attach -t <name>`
then just work.
Shared shell helpers live in `lib.sh`; keep `init_cmd` sourcing it.
