"""One-time unchanged-runtime continuation; --check-only never launches."""
import difflib
import json
import sys
from pathlib import Path

root = Path.cwd()
results = root / 'outputs/paper1-final-eval-v1'
original = results / 'operator/serial-execution-recipe.py'
source = original.read_text()
old_source = source
invalid = json.loads((results / 'slot-155-eval-hard-001-no_occlusion/attempt/attempt.json').read_text())
assert invalid['status'] == 'INVALID_TRIAL' and invalid['task_started'] is False
assert invalid['reason'] == 'RuntimeError: SIM exited before readiness'
assert 'finish_wall' in invalid and invalid.get('retrieval_success') is None
assert not (results / 'slot-155-eval-hard-001-no_occlusion-replacement-1').exists()
entries = [json.loads(p.read_text()) for p in results.glob('slot-*/entry.json')]
assert len(entries) == 156 and max(e['slot'] for e in entries) == 155
assert min(e['start_wall'] for e in entries) == 1789063252.3554409
report = json.loads((results / 'analysis-latest.json').read_text())
assert report['counts']['valid_completed_slots'] == 154
assert report['counts']['invalid_activations'] == 2
assert report['counts']['replacement_starts'] == 1
assert not report['protocol_issues']

changes = [
    ("for spec in config['slots'][4-1:]:", "for spec in config['slots'][155-1:]:"),
    ("operators/'progress.jsonl'", "operators/'progress-after-startup-155.jsonl'"),
    ("if len(entries)>=224", "if len(entries)+1>=224"),
    ("starts=len(entries),**resource", "starts=len(entries)+1,**resource"),
    ("starts_before=len(entries),**resource", "starts_before=len(entries),external_bare_starts=1,total_starts_before=len(entries)+1,**resource"),
    ("    directory=Path(command[command.index('--output-dir')+1])",
     "    directory=Path(command[command.index('--output-dir')+1])\n"
     "    if slot == 155:\n"
     "        directory=directory.with_name(directory.name+'-replacement-1')\n"
     "        command[command.index('--output-dir')+1]=str(directory)"),
    ("with (operators/('slot-%03d-console.log'%slot)).open('x') as log:",
     "with (operators/(('slot-%03d-replacement-1-console.log' if slot == 155 "
     "else 'slot-%03d-console.log')%slot)).open('x') as log:"),
    ("        except KeyboardInterrupt:",
     "        except (KeyboardInterrupt, OSError, subprocess.CalledProcessError):"),
]
for old, new in changes:
    assert source.count(old) == 1, old
    source = source.replace(old, new)
compile(source, str(original), 'exec')
if '--check-only' in sys.argv:
    print('OFFLINE ONLY: one invalid slot 155 replacement, then original slots 156..212')
    print('Original deadline 1789495252.3554409; +1 external bare start counted toward 224')
    print(''.join(difflib.unified_diff(old_source.splitlines(True), source.splitlines(True),
                                     fromfile='original operator', tofile='continuation operator')))
else:
    sys.path.insert(0, str(root / 'scripts'))
    exec(compile(source, str(original), 'exec'))
