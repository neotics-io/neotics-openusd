# Neotics kit for OpenUSD

This folder is Neotics' own work, added to our copy of [OpenUSD](https://github.com/PixarAnimationStudios/OpenUSD). It turns any Neotics scene into a USD stage and animates the arm from a recorded run.

It is one part of the Neotics stack. Every fork in [neotics-io](https://github.com/neotics-io) reads the same two files:

- a **Neotics Scene** (`samples/depot.scene.json`): the room, the obstacles, the arm and the named places;
- a **Neotics Episode** (`samples/depot.episode.json`): a run recorded in MuJoCo, with time, joint positions and commands.

The depot is a sample made for these tools. It is not an entry or an answer for any [Neotics challenge](https://lab.neotics.io).

## Use it

Run from this folder:

```bash
python scene_to_usd.py samples/depot.scene.json depot.usda samples/depot.episode.json
```

## How it's checked

Reopens the stage, checks units and every part, and compares the animated tool position with hand-worked kinematics. This runs on GitHub in `.github/workflows/neotics.yml`, on every change to this folder.

## Licence

See [`LICENSE`](LICENSE). It covers this folder only.
