# Contributing

Thank you for your interest in improving the X-Plane G1000 NXi project. Pull
requests, bug reports, and discussion are welcome.

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

1. **Discuss large changes first** — open an issue or comment on an existing one
   before spending time on a major refactor or new feature.
2. **Fork and branch** — work on a feature branch off `main`.
3. **Build and test** — see [README.md](README.md) and [AGENTS.md](AGENTS.md).
   Run `ctest` when your change touches logic covered by unit tests.
4. **Open a pull request** — describe what changed and why. Link any related
   issues.

## Freeware aircraft developers

You may bundle or integrate this avionics suite into **freeware** (non-commercial)
X-Plane aircraft under the PolyForm Noncommercial License. Include a copy of
[LICENSE](LICENSE) with your distribution and retain the `Required Notice` line.
Payware or other commercial distribution requires a commercial license — see
[COMMERCIAL-LICENSE.md](COMMERCIAL-LICENSE.md).

## Code style

Match the surrounding code: naming, formatting, and structure. Keep changes
focused on the problem you are solving.
