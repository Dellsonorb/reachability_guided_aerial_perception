"""One-time continuation after resolved offline cleanup; --check-only never launches."""
import difflib
import json
import sys
from pathlib import Path

root = Path.cwd()
results = root / 'outputs/paper1-final-eval-v1'
original = results / 'operator/serial-execution-recipe.py'
source = original.read_text()
old_source = source
entries = [json.loads(p.read_text()) for p in results.glob('slot-*/entry.json')]
assert len(entries) == 133 and max(e['slot'] for e in entries) == 132
assert min(e['start_wall'] for e in entries) == 1789063252.3554409
report = json.loads((results / 'analysis-latest.json').read_text())
assert report['counts']['valid_completed_slots'] == 132
assert report['counts']['invalid_activations'] == 1
assert report['counts']['replacement_starts'] == 1
assert not report['protocol_issues']
for name in ['slot-131-eval-easy-018-ours-replacement-1', 'slot-132-eval-easy-018-generic']:
    assert (results / name / 'attempt/retention.json').exists()

changes = [
    ("for spec in config['slots'][4-1:]:", "for spec in config['slots'][133-1:]:"),
    ("operators/'progress.jsonl'", "operators/'progress-after-slot132-cleanup.jsonl'"),
    ("if len(entries)>=224", "if len(entries)+1>=224"),
    ("starts=len(entries),**resource", "starts=len(entries)+1,**resource"),
    ("starts_before=len(entries),**resource", "starts_before=len(entries),external_bare_starts=1,total_starts_before=len(entries)+1,**resource"),
    ("        except KeyboardInterrupt:",
     "        except (KeyboardInterrupt, OSError, subprocess.CalledProcessError):"),
]
for old, new in changes:
    assert source.count(old) == 1, old
    source = source.replace(old, new)
compile(source, str(original), 'exec')
if '--check-only' in sys.argv:
    print('OFFLINE ONLY: original slots 133..212; completed slots are not rerun')
    print('Original deadline 1789495252.3554409; +1 external bare start counted toward 224')
    print(''.join(difflib.unified_diff(old_source.splitlines(True), source.splitlines(True),
                                     fromfile='original operator', tofile='continuation operator')))
else:
    sys.path.insert(0, str(root / 'scripts'))
    exec(compile(source, str(original), 'exec'))
