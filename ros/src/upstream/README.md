# Upstream sources

Everything in this folder except this README is imported by vcstool and ignored by git:

```bash
cd ros
vcs import src/upstream < upstream.repos && ./patches/apply.sh
```

Don't edit these checkouts. Changes we need go into our own packages (config, launch, adapter
nodes) or, when that isn't possible, into a patch in `ros/patches/`.
