# Datasheets

This directory holds the reference documents for every piece of silicon, board, or actuator in owlet. The PDFs themselves are **not tracked in git** (see `.gitignore`). Run `fetch_datasheets.sh` to download the current versions into this folder.

Each document in `INDEX.md` that can be fetched by script has an entry in `sources.env`. Update `sources.env` and re-run the fetch script to pull a newer revision.
