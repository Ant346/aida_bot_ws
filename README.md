# aida_bot_ws

ROS 2 workspace. This repository uses **Git submodules** for vendored packages; clone them so paths like `ds4_driver_submodule/` and `src/realsense-ros/` are populated.

## Clone (recommended)

Clone the repository and initialize submodules in one step:

```bash
git clone --recurse-submodules https://github.com/Ant346/aida_bot_ws.git
cd aida_bot_ws
```

Shallow clones are fine if you only need current commits:

```bash
git clone --recurse-submodules --depth 1 https://github.com/Ant346/aida_bot_ws.git
cd aida_bot_ws
```

## Already cloned without submodules

From the repo root:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

After `git pull` on the parent repo, submodule pointers may move; refresh checkouts:

```bash
git submodule update --init --recursive
```

## Submodules

| Path | Repository | Configured branch (for `update --remote`) |
|------|------------|-------------------------------------------|
| `ds4_driver_submodule` | [naoki-mizuno/ds4_driver](https://github.com/naoki-mizuno/ds4_driver) | `humble` |
| `src/realsense-ros` | [realsenseai/realsense-ros](https://github.com/realsenseai/realsense-ros) | `ros2-master` |

Commits are pinned by this repo’s superproject; **`git submodule update`** checks out those pins. Use **`git submodule update --remote`** only if you intentionally want to move to the latest commit on the branch above (then commit the new submodule SHA in the parent repo).

## Verify

```bash
git submodule status
```

Each line should start with a space (submodule checked out at the expected commit), not `-` (missing) or `+` (different commit than recorded).
