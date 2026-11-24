#!/usr/bin/env python3
"""Count likely BARR-C:2018 rule breaks in the team's C code (#60).

This is a heuristic line scanner, not a parser or a certified checker.
It strips comments and string literals, then counts patterns that break
rules a script can test. Each count is a lead to look at, not a proven
violation: expect a few false positives per rule.

Usage:
    python tools/barr_c_check.py                 # summary for app_program/
    python tools/barr_c_check.py --files         # plus a per-file table
    python tools/barr_c_check.py --show 6.2-len  # list every hit for one check
    python tools/barr_c_check.py app_program/subsystem2

Line endings are read from the index (git ls-files --eol), so a Windows
checkout with autocrlf does not count as CRLF.
"""

import argparse
import collections
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# check id -> (BARR-C section, short description)
CHECKS = collections.OrderedDict(
    [
        ("1.2-width", ("1.2", "line longer than 80 characters")),
        ("1.3-brace", ("1.3", "left brace not on a line by itself")),
        ("1.3-nobrace", ("1.3", "if/else/for/while body without braces")),
        ("1.7-goto", ("1.7", "goto used")),
        ("1.7-continue", ("1.7", "continue used")),
        ("1.7-auto-reg", ("1.7", "auto or register used")),
        ("3.4-indent", ("3.4", "code indent not a multiple of 4 spaces")),
        ("3.5-tab", ("3.5", "tab character in a line")),
        ("3.6-crlf", ("3.6", "file stored with CRLF line endings")),
        ("4.2-guard", ("4.2", "header without an #ifndef include guard")),
        ("4.3-inc-c", ("4.3", "#include of a .c file")),
        ("5.1-typedef", ("5.1", "typedef name does not end in _t")),
        ("5.2-int", ("5.2", "plain int / INT / UINT for numeric data")),
        ("5.2-short-long", ("5.2", "short or long keyword used")),
        ("6.1-fn-case", ("6.1", "function name contains uppercase")),
        ("6.1-macro-case", ("6.1", "macro name contains lowercase")),
        ("6.2-len", ("6.2", "function body longer than 100 lines")),
        ("6.2-exit", ("6.2", "function with more than one return")),
        ("6.3-fnmacro", ("6.3", "function-like macro defined")),
        ("7.1-short", ("7.1", "variable name shorter than 3 characters")),
        ("7.1-case", ("7.1", "variable name contains uppercase")),
        ("7.1-global", ("7.1", "file-scope variable without g_ prefix")),
        ("7.1-pointer", ("7.1", "pointer variable without p_ prefix")),
        ("7.1-bool", ("7.1", "Boolean variable without b_ prefix")),
        ("8.2-else", ("8.2", "if ... else if chain without a final else")),
        ("8.3-default", ("8.3", "switch without a default case")),
        ("8.5-exit", ("8.5", "setjmp/longjmp/abort/exit used")),
        ("8.6-const-right", ("8.6", "== or != with the constant on the right")),
    ]
)

C_KEYWORDS = {
    "if",
    "else",
    "for",
    "while",
    "do",
    "switch",
    "case",
    "default",
    "return",
    "break",
    "continue",
    "goto",
    "sizeof",
    "typedef",
    "struct",
    "union",
    "enum",
    "static",
    "const",
    "volatile",
    "extern",
    "inline",
    "void",
    "char",
    "short",
    "int",
    "long",
    "float",
    "double",
    "signed",
    "unsigned",
    "auto",
    "register",
    "restrict",
    "_Bool",
    "bool",
}

# Type names a local or file-scope declaration can start with. The
# T-Kernel names come from include/tk/typedef.h.
TYPE_WORDS = (
    r"(?:const\s+|volatile\s+|static\s+|unsigned\s+|signed\s+)*"
    r"(?:struct\s+\w+|enum\s+\w+|union\s+\w+|u?int(?:8|16|32|64)_t|"
    r"size_t|bool|char|int|float|double|long|short|unsigned|"
    r"B|H|W|D|UB|UH|UW|UD|INT|UINT|BOOL|ER|ID|TMO|SYSTIM|RELTIM|SZ|"
    r"ATR|PRI|FLGPTN|[A-Z][A-Z0-9_]*_t|[a-z][a-z0-9_]*_t|[A-Z][A-Z0-9_]+)"
)
DECL_RE = re.compile(
    r"^\s*(?P<type>" + TYPE_WORDS + r")(?:\s*(?P<stars>\*+\s*(?:const\s+)?)|\s+)"
    r"(?P<name>[A-Za-z_]\w*)\s*(?:\[[^\]]*\]\s*)*(?:=|;|,)"
)
FUNC_DEF_RE = re.compile(
    r"^(?P<head>[A-Za-z_][\w\s\*]*?)\b(?P<name>[A-Za-z_]\w*)\s*\([^;{}]*\)\s*\{?\s*$"
)
CONST_RE = r"(?:NULL|TRUE|FALSE|true|false|E_[A-Z]+|[A-Z][A-Z0-9_]{2,}|0[xX][0-9a-fA-F]+[uUlL]*|\d+[uUlLfF]*|'(?:\\.|[^'])')"
CONST_RIGHT_RE = re.compile(r"[\w\)\]]\s*(?:==|!=)\s*" + CONST_RE + r"\b")
CONST_RIGHT_RE2 = re.compile(r"[\w\)\]]\s*(?:==|!=)\s*'")


def strip_code(text):
    """Blank comments, string and char literals, keeping line structure.

    Char literals keep their quotes so 8.6 can still see them as constants.
    """
    out = []
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", text[i:j]))
            i = j
        elif text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif c == '"' or c == "'":
            j = i + 1
            while j < n and text[j] != c and text[j] != "\n":
                j += 2 if text[j] == "\\" else 1
            j = min(j + 1, n)
            body = re.sub(r"[^\n]", " ", text[i + 1 : j - 1])
            out.append(c + body + c if c == "'" else '""' + body)
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def git_crlf_files():
    """Paths whose index copy has CRLF line endings."""
    try:
        res = subprocess.run(
            ["git", "-C", ROOT, "ls-files", "--eol"],
            capture_output=True,
            text=True,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    crlf = set()
    for line in res.stdout.splitlines():
        parts = line.split(None, 3)
        if len(parts) == 4 and parts[0] in ("i/crlf", "i/mixed"):
            crlf.add(os.path.normpath(parts[3]))
    return crlf


def find_block_end(code, open_idx):
    depth = 0
    for k in range(open_idx, len(code)):
        if code[k] == "{":
            depth += 1
        elif code[k] == "}":
            depth -= 1
            if depth == 0:
                return k
    return len(code) - 1


def line_of(code, idx):
    return code.count("\n", 0, idx) + 1


def skip_body(code, k):
    """From index k (start of an if body), return the index just after it.

    Only the body itself is skipped, never a following else.
    """
    while k < len(code) and code[k].isspace():
        k += 1
    if k < len(code) and code[k] == "{":
        return find_block_end(code, k) + 1
    m = re.compile(r"(if|for|while)\s*\(").match(code, k)
    if m:
        k = match_paren(code, m.end() - 1) + 1
        k = skip_body(code, k)
        if m.group(1) == "if":
            m2 = re.compile(r"\s*else\b").match(code, k)
            if m2:
                return skip_body(code, m2.end())
        return k
    j = code.find(";", k)
    return len(code) if j < 0 else j + 1


def match_paren(code, open_idx):
    depth = 0
    for k in range(open_idx, len(code)):
        if code[k] == "(":
            depth += 1
        elif code[k] == ")":
            depth -= 1
            if depth == 0:
                return k
    return len(code) - 1


def scan_file(path, rel, crlf_files, hits):
    with open(path, encoding="utf-8", errors="replace", newline="") as f:
        raw = f.read()
    raw_lines = raw.replace("\r\n", "\n").split("\n")
    code = strip_code(raw.replace("\r\n", "\n"))
    lines = code.split("\n")
    is_header = rel.endswith(".h")

    def hit(check, lineno, text=""):
        hits[check].append((rel, lineno, text.strip()[:70]))

    if crlf_files is not None and os.path.normpath(rel) in crlf_files:
        hit("3.6-crlf", 1)

    in_macro = False
    for no, (rl, cl) in enumerate(zip(raw_lines, lines), 1):
        if len(rl.expandtabs(4)) > 80:
            hit("1.2-width", no, rl)
        if "\t" in rl:
            hit("3.5-tab", no, rl)
        continuing_macro = in_macro
        in_macro = cl.rstrip().endswith("\\") and (
            continuing_macro or cl.lstrip().startswith("#")
        )
        stripped = cl.strip()
        if not stripped or continuing_macro or stripped.startswith("#"):
            pass
        else:
            indent = len(cl) - len(cl.lstrip(" "))
            if (
                not cl.startswith("\t")
                and indent % 4 != 0
                and not re.match(r"^[\)\]\}&|+\-*/?:<>=,.]", stripped)
                and prev_code_ends_statement(lines, no)
            ):
                hit("3.4-indent", no, rl)
        # 1.3: '{' that shares its line with code (not initializers).
        if "{" in stripped and stripped != "{" and not stripped.startswith("#"):
            before = stripped.split("{", 1)[0].rstrip()
            if (
                before
                and not before.endswith(("=", ",", "(", "{", "return"))
                and not re.search(r"\b(enum|struct|union)\b[^=]*=", before)
            ):
                hit("1.3-brace", no, rl)
        for kw, check in (
            ("goto", "1.7-goto"),
            ("continue", "1.7-continue"),
            ("auto", "1.7-auto-reg"),
            ("register", "1.7-auto-reg"),
        ):
            if re.search(r"\b" + kw + r"\b", cl):
                hit(check, no, rl)
        if re.search(r"#\s*include\s*[\"<][^\">]+\.c[\">]", rl):
            hit("4.3-inc-c", no, rl)
        if re.search(r"\b(short|long)\b", cl):
            hit("5.2-short-long", no, rl)
        if re.search(r"(?<![\w.])(int|INT|UINT)\b(?!\s*\()", cl) and not re.search(
            r"\bint\s+main\b", cl
        ):
            hit("5.2-int", no, rl)
        if re.search(r"\b(setjmp|longjmp|abort|exit)\s*\(", cl):
            hit("8.5-exit", no, rl)
        if CONST_RIGHT_RE.search(cl) or CONST_RIGHT_RE2.search(cl):
            hit("8.6-const-right", no, rl)
        m = re.match(r"\s*#\s*define\s+(\w+)(\()?", cl)
        if m:
            if m.group(2):
                hit("6.3-fnmacro", no, rl)
            if re.search(r"[a-z]", m.group(1)):
                hit("6.1-macro-case", no, rl)
        m = re.search(r"\btypedef\b.*?\b(\w+)\s*(?:\[[^\]]*\])?\s*;\s*$", cl)
        if m and not m.group(1).endswith("_t") and "(" not in cl:
            hit("5.1-typedef", no, rl)
        m = re.match(r"\s*\}\s*(\w+)\s*;", cl)
        if m and m.group(1) not in C_KEYWORDS and not m.group(1).endswith("_t"):
            # closing line of a multi-line typedef struct/enum
            start = code.rfind("typedef", 0, sum(len(x) + 1 for x in lines[: no - 1]))
            if (
                start >= 0
                and code.count("{", start, sum(len(x) + 1 for x in lines[:no])) > 0
            ):
                hit("5.1-typedef", no, rl)

    # 4.2 include guard
    if (
        is_header
        and not re.search(r"^\s*#\s*ifndef\s+\w+\s*\n\s*#\s*define\s+\w+", code, re.M)
        and "#pragma once" not in code
    ):
        hit("4.2-guard", 1)

    # Function definitions: length, returns, name case, bodies.
    depth = 0
    k = 0
    n = len(code)
    while k < n:
        c = code[k]
        if c == "{":
            if depth == 0:
                # Look back for "name(...)" right before this brace.
                head_start = (
                    max(
                        code.rfind(";", 0, k),
                        code.rfind("}", 0, k),
                        code.rfind("#", 0, k),
                    )
                    + 1
                )
                if code[head_start - 1 : head_start] == "#":
                    head_start = code.find("\n", head_start) + 1
                head = code[head_start:k].strip()
                m = re.search(r"\b([A-Za-z_]\w*)\s*\(([^()]|\([^()]*\))*\)\s*$", head)
                end = find_block_end(code, k)
                if (
                    m
                    and not re.search(r"\b(struct|union|enum)\b\s*\w*\s*$", head)
                    and "=" not in head.split("(")[0]
                ):
                    name = m.group(1)
                    start_line = line_of(code, k)
                    end_line = line_of(code, end)
                    body = code[k : end + 1]
                    if end_line - start_line - 1 > 100:
                        hit(
                            "6.2-len",
                            start_line,
                            "%s(): %d lines" % (name, end_line - start_line - 1),
                        )
                    nret = len(re.findall(r"\breturn\b", body))
                    if nret > 1:
                        hit("6.2-exit", start_line, "%s(): %d returns" % (name, nret))
                    if re.search(r"[A-Z]", name) and name != "main":
                        hit("6.1-fn-case", start_line, name)
                    scan_body(code, k, end, rel, hit)
                    k = end + 1
                    continue
                k = end + 1
                continue
        elif c == ";" and depth == 0:
            pass
        k += 1

    # File-scope variables (outside any brace).
    depth = 0
    pos = 0
    for no, cl in enumerate(lines, 1):
        if depth == 0 and not cl.lstrip().startswith(("#", "typedef", "extern")):
            m = DECL_RE.match(cl)
            # A line ending in "," or ")" without "=" is a parameter list.
            if (
                m
                and "(" not in cl.split("=")[0]
                and m.group("name") not in C_KEYWORDS
                and (cl.rstrip().endswith((";", "{")) or "=" in cl)
            ):
                name = m.group("name")
                if not name.startswith("g_") and not re.fullmatch(r"[A-Z0-9_]+", name):
                    hit("7.1-global", no, cl)
                check_var_name(name, m.group("stars"), no, cl, hit, m.group("type"))
        depth += cl.count("{") - cl.count("}")
        if depth < 0:
            depth = 0


def prev_code_ends_statement(lines, no):
    """True if the previous non-blank line ends a statement or block, so
    line `no` starts a statement (not a continuation, which 3.2 lets align)."""
    for k in range(no - 2, -1, -1):
        prev = lines[k].strip()
        if prev:
            if re.match(r"(case\b|default\b|\w+\s*:$)", prev):
                return True  # case label or goto label
            return prev.startswith("#") or prev.endswith((";", "{", "}"))
    return True


def check_var_name(name, stars, no, cl, hit, type_name=""):
    bare = re.sub(r"^(g_|s_)", "", name)
    if len(name) < 3:
        hit("7.1-short", no, cl)
    if re.search(r"[A-Z]", name) and not re.fullmatch(r"[A-Z0-9_]+", name):
        hit("7.1-case", no, cl)
    if stars and stars.count("*") == 1 and not bare.startswith("p_"):
        hit("7.1-pointer", no, cl)
    if (
        not stars
        and re.search(r"\b(bool|BOOL)\b", type_name)
        and not bare.startswith("b_")
    ):
        hit("7.1-bool", no, cl)


def scan_body(code, start, end, rel, hit):
    body = code[start : end + 1]
    base_line = line_of(code, start)
    blines = body.split("\n")
    # local declarations
    for i, cl in enumerate(blines):
        m = DECL_RE.match(cl)
        if (
            m
            and m.group("name") not in C_KEYWORDS
            and not re.match(r"\s*(return|case|else|goto)\b", cl)
        ):
            check_var_name(
                m.group("name"),
                m.group("stars"),
                base_line + i,
                cl,
                hit,
                m.group("type"),
            )
        for m in re.finditer(r"\bfor\s*\(\s*(?:" + TYPE_WORDS + r")\s+(\w+)\s*=", cl):
            if len(m.group(1)) < 3:
                hit("7.1-short", base_line + i, cl)
    # switch without default
    for m in re.finditer(r"\bswitch\s*\(", body):
        p = match_paren(body, m.end() - 1)
        o = body.find("{", p)
        e = find_block_end(body, o)
        inner = body[o + 1 : e]
        # only count default at this switch's own depth
        depth = 0
        has_default = False
        for tok in re.finditer(r"[{}]|\bdefault\s*:", inner):
            t = tok.group(0)
            if t == "{":
                depth += 1
            elif t == "}":
                depth -= 1
            elif depth == 0:
                has_default = True
        if not has_default:
            ln = base_line + body.count("\n", 0, m.start())
            hit("8.3-default", ln, body[m.start() : m.start() + 60].split("\n")[0])
    # if / else if chains without final else; unbraced bodies
    # (?<!#) skips #if directives.
    for m in re.finditer(r"(?<!#)\b(if|while|for)\s*\(", body):
        kw = m.group(1)
        p = match_paren(body, m.end() - 1)
        q = p + 1
        while q < len(body) and body[q] in " \t\n":
            q += 1
        prev = body[max(0, m.start() - 6) : m.start()]
        is_do_while = kw == "while" and re.search(r"\}\s*$", body[: m.start()])
        if q < len(body) and body[q] != "{" and body[q] != ";" and not is_do_while:
            ln = base_line + body.count("\n", 0, m.start())
            hit("1.3-nobrace", ln, body[m.start() : m.start() + 60].split("\n")[0])
        if kw == "if" and re.search(r"else\s+$", prev):
            # an "else if": the statement after its body must be an "else"
            # (a bare else, or another else if that is checked in turn)
            after = skip_body(body, p + 1)
            if not re.compile(r"\s*else\b").match(body, after):
                ln = base_line + body.count("\n", 0, m.start())
                hit("8.2-else", ln, body[m.start() : m.start() + 60].split("\n")[0])
    # (?<!#) skips the #else preprocessor directive.
    for m in re.finditer(r"(?<!#)\belse\b(?!\s*if\b)", body):
        q = m.end()
        while q < len(body) and body[q] in " \t\n":
            q += 1
        if q < len(body) and body[q] != "{":
            ln = base_line + body.count("\n", 0, m.start())
            hit("1.3-nobrace", ln, "else " + body[q : q + 50].split("\n")[0])


def collect(paths):
    files = []
    for p in paths:
        ap = os.path.join(ROOT, p)
        if os.path.isfile(ap):
            files.append(ap)
            continue
        for dirpath, _, names in os.walk(ap):
            for nm in sorted(names):
                if nm.endswith((".c", ".h")):
                    files.append(os.path.join(dirpath, nm))
    return sorted(files)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("paths", nargs="*", default=["app_program"])
    ap.add_argument("--files", action="store_true", help="per-file table")
    ap.add_argument("--show", metavar="CHECK", help="list hits for one check id")
    args = ap.parse_args()

    files = collect(args.paths)
    crlf = git_crlf_files()
    hits = collections.defaultdict(list)
    total_lines = 0
    for path in files:
        rel = os.path.relpath(path, ROOT).replace("\\", "/")
        with open(path, encoding="utf-8", errors="replace") as f:
            total_lines += sum(1 for _ in f)
        scan_file(path, rel, crlf, hits)

    if args.show:
        for rel, no, text in hits.get(args.show, []):
            print("%s:%d: %s" % (rel, no, text))
        return 0

    print("BARR-C:2018 heuristic scan of %s" % ", ".join(args.paths))
    print("%d files, %d lines\n" % (len(files), total_lines))
    print("%-16s %-5s %6s %6s  %s" % ("check", "rule", "hits", "files", "what"))
    for cid, (rule, desc) in CHECKS.items():
        h = hits.get(cid, [])
        nfiles = len({r for r, _, _ in h})
        print("%-16s %-5s %6d %6d  %s" % (cid, rule, len(h), nfiles, desc))

    if args.files:
        print("\nper file (checks with hits):")
        per = collections.defaultdict(collections.Counter)
        for cid, h in hits.items():
            for rel, _, _ in h:
                per[rel][cid] += 1
        for rel in sorted(per):
            items = ", ".join("%s=%d" % (c, per[rel][c]) for c in CHECKS if per[rel][c])
            print("  %s: %s" % (rel, items))
    return 0


if __name__ == "__main__":
    sys.exit(main())
