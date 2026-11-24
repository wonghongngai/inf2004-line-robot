"""INT-14: the event-bit table in TEAM.md section 5 (#50) matches the code.

A static check of the firmware sources (no compiler, no robot). For every
kernel object in the table it finds each call that sets, clears, waits on,
sends to or reads from it, and fails if any of them is in a file the table
does not name. This is the "one producer sets a bit; only the producer or the
one named consumer clears it" rule, checked automatically, so a new writer
cannot be added without updating the table.

Test harnesses (all_test, line_follow_test, line_demo) are separate firmware
images that replace the VC. They are listed as the line-event consumer of
their own image, as TEAM.md section 4 says.
"""
from pathlib import Path
import re
import sys

root = Path(__file__).resolve().parents[2]
# Optional argument: another app_program directory (used for mutation runs on
# a copy, so the working tree is never edited).
app = Path(sys.argv[1]) if len(sys.argv) > 1 else root / 'app_program'
failures = []


def strip_comments(text):
    text = re.sub(r'/\*.*?\*/', ' ', text, flags=re.S)
    return re.sub(r'//[^\n]*', ' ', text)


sources = {p.relative_to(app).as_posix(): strip_comments(p.read_text(encoding='utf-8', errors='replace'))
           for p in sorted(app.rglob('*.c'))}


def calls(func, obj):
    """Every call func(obj, <args>); in the sources: [(file, args)]."""
    pattern = re.compile(r'\b' + func + r'\s*\(\s*' + obj + r'\s*(?:,(.*?))?\)\s*[;)]', re.S)
    found = []
    for name, text in sources.items():
        for m in pattern.finditer(text):
            found.append((name, ' '.join((m.group(1) or '').split())))
    return found


def check(what, found, allowed, minimum=1):
    files = sorted({f for f, _ in found})
    extra = [f for f in files if f not in allowed]
    if extra:
        failures.append(f'{what}: not allowed in {extra} (table allows {sorted(allowed)})')
    if len(found) < minimum:
        failures.append(f'{what}: expected at least {minimum} call(s), found {len(found)}')
    print(f'{"FAIL" if extra or len(found) < minimum else "ok  "} {what}: {files or "none"}')


# ---- STOP bit, g_flg_motion_stop (S2) ---------------------------------------
check('g_flg_motion_stop set (producer: subsystem2_send_command)',
      calls('tk_set_flg', 'g_flg_motion_stop'), {'subsystem2/subsystem2.c'})
check('g_flg_motion_stop cleared with tk_clr_flg (nobody: consumed by TWF_CLR)',
      calls('tk_clr_flg', 'g_flg_motion_stop'), set(), minimum=0)
waits = calls('tk_wai_flg', 'g_flg_motion_stop')
check('g_flg_motion_stop waited on (consumer: motion_task)', waits, {'subsystem2/task_motion.c'})
if not all('TWF_CLR' in args for _, args in waits):
    failures.append('g_flg_motion_stop: the consumer must clear on read (TWF_CLR)')

# ---- S5_EVT_* bits, flg_s5_evt (S5) ---------------------------------------
check('flg_s5_evt set (producer: VC executor)',
      calls('tk_set_flg', 'flg_s5_evt'), {'subsystem5/s5_executor.c'})
clears = calls('tk_clr_flg', 'flg_s5_evt')
check('flg_s5_evt cleared (VC: FOLLOWING; S5: its reply bits)', clears,
      {'subsystem5/s5_executor.c', 'subsystem5/task_obstacle.c'})
waits = calls('tk_wai_flg', 'flg_s5_evt')
check('flg_s5_evt waited on (consumer: S5 obstacle task)', waits, {'subsystem5/task_obstacle.c'})
# tk_clr_flg keeps the bits that are set in its pattern.
for name, args in clears:
    if name == 'subsystem5/s5_executor.c':
        if args != '~S5_EVT_FOLLOWING':
            failures.append(f'{name}: the VC may clear only S5_EVT_FOLLOWING, got tk_clr_flg(..., {args})')
    elif 'FOLLOWING' in args:
        failures.append(f'{name}: S5 must not clear S5_EVT_FOLLOWING (owned by the VC): {args}')
# S5 clears "~wanted": wanted must never include FOLLOWING.
for m in re.finditer(r'\bwanted\s*\|?=([^;]*);', sources['subsystem5/task_obstacle.c']):
    if 'FOLLOWING' in m.group(1):
        failures.append(f'task_obstacle.c: a reply mask includes S5_EVT_FOLLOWING: {m.group(1).strip()}')
# TWF_CLR would wipe every bit, FOLLOWING included.
for name, args in waits:
    if 'TWF_CLR' in args:
        failures.append(f'{name}: S5 must not wait with TWF_CLR (it would clear the VC\'s FOLLOWING bit)')

# ---- line events, g_h_line_nav_q (S3) -------------------------------------
check('g_h_line_nav_q sent to (producer: line_service_task)',
      calls('tk_snd_mbf', 'g_h_line_nav_q'), {'subsystem3/line_service.c'})
check('g_h_line_nav_q read directly (only through line_service_receive/enable)',
      calls('tk_rcv_mbf', 'g_h_line_nav_q'), {'subsystem3/line_service.c'})
receivers = re.compile(r'\bline_service_receive\s*\(')
readers = sorted(n for n, t in sources.items()
                 if receivers.search(t) and n != 'subsystem3/line_service.c')
allowed_readers = {'subsystem2/robot_control.c',      # the VC (integrated)
                   'all_test/all_buddies_test.c',     # all_test image
                   'all_test/line_follow_test.c',     # line_follow_test image
                   'subsystem3/line_demo.c'}          # barcode_test image
extra = [n for n in readers if n not in allowed_readers]
print(f'{"FAIL" if extra else "ok  "} line events read (one consumer per image): {readers}')
if extra:
    failures.append(f'line events read by {extra}: one consumer per image (TEAM.md section 5)')

# ---- telemetry fault bits (IF-05) -----------------------------------------
fault_writers = sorted(n for n, t in sources.items()
                       if re.search(r'\bS1_FAULT_\w+', t) and not n.startswith('subsystem1/'))
extra = [n for n in fault_writers
         if n not in {'subsystem2/robot_control.c', 'all_test/all_buddies_test.c'}]
print(f'{"FAIL" if extra else "ok  "} S1_FAULT_* set outside S1: {fault_writers}')
if extra:
    failures.append(f'S1_FAULT_* bits set by {extra}: the table names only the VC and all_test')

if failures:
    print('\nFAIL')
    for f in failures:
        print('  ' + f)
    sys.exit(1)
print('\nevent-bit table vs code: PASS')
