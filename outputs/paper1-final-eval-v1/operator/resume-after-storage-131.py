"""One-time operator continuation; never import or rerun blindly.

No robot code/configuration changes. Preserve the interrupted recipe and logs.
Run --check-only for an offline diff without launching a task.
"""
import difflib
import json
import sys
from pathlib import Path

root = Path.cwd()
results = root / 'outputs/paper1-final-eval-v1'
operator = results / 'operator'
original = operator / 'serial-execution-recipe.py'
source = original.read_text()
original_source = source
invalid = json.loads((results / 'slot-131-eval-easy-018-ours/attempt/attempt.json').read_text())
assert invalid['status'] == 'INVALID_TRIAL' and invalid['reason']
assert invalid['retrieval_success'] is None and 'finish_wall' in invalid
assert not (results / 'slot-131-eval-easy-018-ours-replacement-1').exists()
entries = [json.loads(p.read_text()) for p in results.glob('slot-*/entry.json')]
assert len(entries) == 131 and max(e['slot'] for e in entries) == 131
assert min(e['start_wall'] for e in entries) == 1789063252.3554409

changes = [
    ("for spec in config['slots'][4-1:]:", "for spec in config['slots'][131-1:]:"),
    ("operators/'progress.jsonl'", "operators/'progress-after-storage-131.jsonl'"),
    (
        "    directory=Path(command[command.index('--output-dir')+1])",
        "    directory=Path(command[command.index('--output-dir')+1])\n"
        "    if slot == 131:\n"
        "        directory=directory.with_name(directory.name+'-replacement-1')\n"
        "        command[command.index('--output-dir')+1]=str(directory)",
    ),
    (
        "with (operators/('slot-%03d-console.log'%slot)).open('x') as log:",
        "with (operators/(('slot-%03d-replacement-1-console.log' if slot == 131 "
        "else 'slot-%03d-console.log')%slot)).open('x') as log:",
    ),
    (
        "        except KeyboardInterrupt:",
        "        except (KeyboardInterrupt, OSError, subprocess.CalledProcessError):",
    ),
]
for old, new in changes:
    assert source.count(old) == 1, old
    source = source.replace(old, new)
compile(source, str(original), 'exec')

if '--check-only' in sys.argv:
    print('OFFLINE ONLY: replacement of invalid slot 131, then original slots 132..212')
    print('Original deadline: 1789495252.3554409; maximum starts: 224')
    print(''.join(difflib.unified_diff(original_source.splitlines(True), source.splitlines(True),
                                     fromfile='original operator', tofile='continuation operator')))
else:
    sys.path.insert(0, str(root / 'scripts'))
    exec(compile(source, str(original), 'exec'))
