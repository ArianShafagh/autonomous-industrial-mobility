#!/usr/bin/env python3
"""Verify every reference in thesis/refs.bib against the publisher's Crossref record.

    venv/bin/python tools/thesis/check_refs.py            # report only
    venv/bin/python tools/thesis/check_refs.py --json out.json

For each entry with a DOI it fetches the Crossref record and compares title, year, venue, volume,
issue, pages and the number of authors. Entries without a DOI are listed so they can be checked by
hand. Nothing is written to refs.bib: this tool reports, so that a citation is never "fixed" into
something the publisher does not say.
"""
import argparse
import difflib
import json
import pathlib
import re
import time
import urllib.request

WS = pathlib.Path(__file__).resolve().parents[2]
BIB = WS / "thesis" / "refs.bib"
UA = {"User-Agent": "thesis-reference-check/1.0 (mailto:arian.shafagh2003@gmail.com)"}


def field(body, name):
    m = re.search(rf"\n\s*{name}\s*=\s*\{{+(.*?)\}}+,?\s*\n", body + "\n", re.S)
    return " ".join(m.group(1).split()) if m else None


def normalise(text):
    text = re.sub(r"[{}\\$]", "", text or "").lower()
    return re.sub(r"[^a-z0-9 ]+", " ", text).split()


def crossref(doi):
    request = urllib.request.Request(f"https://api.crossref.org/works/{doi}", headers=UA)
    with urllib.request.urlopen(request, timeout=25) as response:
        return json.load(response)["message"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", default=None, help="also write the fetched records here")
    args = ap.parse_args()

    entries = re.findall(r"@(\w+)\{([^,]+),(.*?)\n\}", BIB.read_text(), re.S)
    records, problems = {}, 0
    for kind, key, body in entries:
        doi = field(body, "doi")
        if not doi:
            print(f"NODOI {key:30s} check by hand")
            continue
        try:
            message = crossref(doi)
        except Exception as exc:                       # network or unknown DOI
            print(f"FAIL  {key:30s} {doi}: {exc}")
            problems += 1
            continue
        record = {"title": (message.get("title") or [""])[0],
                  "year": str((message.get("issued", {}).get("date-parts") or [[None]])[0][0]),
                  "venue": message.get("container-title") or [],
                  "volume": message.get("volume"), "issue": message.get("issue"),
                  "page": message.get("page"),
                  "authors": [f"{a.get('family','')}, {a.get('given','')}".strip(", ")
                              for a in message.get("author", [])]}
        records[key] = record
        ratio = difflib.SequenceMatcher(None, normalise(field(body, "title")),
                                        normalise(record["title"])).ratio()
        flags = []
        if ratio < 0.85:
            flags.append(f"TITLE differs ({ratio:.2f}): {record['title']}")
        if field(body, "year") != record["year"]:
            flags.append(f"YEAR bib={field(body, 'year')} crossref={record['year']}")
        if record["volume"] and not field(body, "volume"):
            flags.append(f"volume missing ({record['volume']})")
        if record["page"] and not field(body, "pages"):
            flags.append(f"pages missing ({record['page']})")
        in_bib = len(re.split(r"\band\b", field(body, "author") or ""))
        if len(record["authors"]) != in_bib:
            flags.append(f"authors bib={in_bib} crossref={len(record['authors'])}")
        problems += bool(flags)
        print(f"{'OK   ' if not flags else 'CHECK'} {key:30s} "
              f"{'; '.join(flags) if flags else (record['venue'] or [''])[0][:45]}")
        time.sleep(0.35)

    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(records, indent=1))
    print(f"\n{len(records)} of {len(entries)} entries verified; {problems} need attention")


if __name__ == "__main__":
    main()
