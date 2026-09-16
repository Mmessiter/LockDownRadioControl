# Licensing

Everything in this repository — the V1 transmitter and receiver firmware, the
circuits, the STL files, the Nextion screens, the documentation — is free
software: **GPL-2.0-or-later** (see [LICENSE](LICENSE)).

Copyright (C) 2024–2026 Malcolm Messiter.

Use it, copy it, change it, share it. If you share a changed version, you must
share its source under the same licence. Derivatives stay free — that is the
point.

## Why version 2 "or later", not version 3

The radio driver this code depends on, [RF24](https://github.com/nRF24/RF24),
is licensed GPL-2.0 **only**. A program that links it must be distributable under
GPL-2.0 terms, and GPL-3.0 code cannot be. "Or later" leaves the door open: if
RF24 is ever replaced, the code can move to GPL-3.0 to match the
[LDRC V2](https://github.com/Mmessiter/LDRC_V2) repository.

## Third-party libraries

Each library keeps its own licence. None of them conflicts with GPL-2.0-or-later
(checked 2026-09-16: no Apache-2.0 or GPL-3.0-only code is in this repository).
The one that decides the version is RF24 (GPL-2.0-only), as above.

## How to comply

- Keep this notice and the copyright line; add your own name for your changes.
- If you distribute a build, distribute the matching source too.
