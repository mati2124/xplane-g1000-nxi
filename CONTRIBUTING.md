# Contributing

Thank you for your interest in improving the X-Plane G1000 NXi project. Pull
requests, bug reports, and discussion are welcome.

**Forking is disabled** on this repository. You can still clone and read the
source, but GitHub will not create personal fork copies. See
[How to contribute](#how-to-contribute) below for the supported workflows.

## License

This project is **source available** under the
[PolyForm Noncommercial License 1.0.0](LICENSE). It is not OSI-approved open
source: commercial use requires a separate agreement (see
[COMMERCIAL-LICENSE.md](COMMERCIAL-LICENSE.md)).

By contributing, you agree that your contributions will be licensed under the
same terms as the rest of the project.

## Developer Certificate of Origin (DCO)

We use the [Developer Certificate of Origin](https://developercertificate.org/)
version 1.1. By signing off your commits, you certify that you have the right to
submit the work under the project's license.

Add a `Signed-off-by` line to every commit message:

```
Signed-off-by: Your Name <your.email@example.com>
```

Use `git commit -s` to add this line automatically.

## How to contribute

### Everyone

1. **Open an issue first for large changes** — describe the bug, feature, or
   refactor before spending time on a major patch. Comment on existing issues
   for smaller fixes.
2. **Bug reports and ideas** — issues are always welcome; no repository write
   access required.

### If you have write access (collaborator)

Forking is off, so work **in this repository** on a branch:

1. Clone the repo (do not fork):
   `git clone https://github.com/andywmm9-pixel/xplane-g1000-nxi.git`
2. Create a feature branch from `main`:
   `git checkout -b your-name/short-description`
3. **Build and test** — see [README.md](README.md) and [AGENTS.md](AGENTS.md).
   Run `ctest` when your change touches logic covered by unit tests.
4. Commit with DCO sign-off (`git commit -s`).
5. Push the branch to **this** repo and open a pull request against `main`.

### If you do not have write access

You cannot push branches or open pull requests until you are added as a
collaborator. To contribute code:

1. **Open an issue** describing the change, or find an issue you want to work
   on and ask to be assigned.
2. **Request collaborator access** in the issue if you plan to submit a pull
   request. Maintainers can grant write access so you can push a branch here
   (no fork needed).
3. **Alternatively, attach a patch** — post a unified diff or link a gist in
   the issue. Maintainers can apply it and credit you in the commit message.

Do not create an unofficial mirror fork on another GitHub account to bypass this
policy; use one of the paths above instead.

## Freeware aircraft developers

You may bundle or integrate this avionics suite into **freeware** (non-commercial)
X-Plane aircraft under the PolyForm Noncommercial License. Include a copy of
[LICENSE](LICENSE) with your distribution and retain the `Required Notice` line.
Payware or other commercial distribution requires a commercial license — see
[COMMERCIAL-LICENSE.md](COMMERCIAL-LICENSE.md).

Cloning or downloading a release tarball for integration is fine; you do not
need a GitHub fork for that.

## Code style

Match the surrounding code: naming, formatting, and structure. Keep changes
focused on the problem you are solving.

## Repository settings (maintainers)

Forking must be turned off in GitHub repository settings (the Cloud Agent token
cannot change this automatically):

1. Open **Settings → General** for
   [andywmm9-pixel/xplane-g1000-nxi](https://github.com/andywmm9-pixel/xplane-g1000-nxi).
2. Under **Features**, uncheck **Allow forking** (or set **Allow forking** to
   off).
3. Save.

Or from a machine with admin access to the repo:

```bash
gh repo edit andywmm9-pixel/xplane-g1000-nxi --allow-forking=false
```

Verify:

```bash
gh api repos/andywmm9-pixel/xplane-g1000-nxi --jq .allow_forking
# should print: false
```

When forking is disabled, add outside contributors as **collaborators**
(**Settings → Collaborators**) if they need to open pull requests directly.
