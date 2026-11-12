"""Check bundled asset/task hashes and confirm recorded rack results match inputs."""
import csv,hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    asset_count=task_count=0
    for name in ['ur5e','ur5e_robotiq']:
        root=ROOT/'assets'/name;provenance=json.loads((root/'provenance.json').read_text())
        for filename,digest in provenance['files'].items():
            assert sha(root/filename)==digest,(name,filename);asset_count+=1
        for urdf in root.glob('*.urdf'):
            for mesh in ET.parse(urdf).getroot().iter('mesh'):
                assert (root/mesh.get('filename')).is_file(),(urdf,mesh.get('filename'))
    for version in ['v0.1','rack_v0.2']:
        root=ROOT/'data'/version;manifest=json.loads((root/'manifest.json').read_text())
        for entry in manifest['tasks']:
            assert sha(root/entry['file'])==entry['sha256'],entry['id']
            witness=entry.get('witness_file',f"reference_witnesses/{entry.get('split','')}/{entry['id']}.npy")
            assert sha(root/witness)==entry['witness_sha256'],entry['id'];task_count+=1
    root=ROOT/'data/rack_v0.2';m=json.loads((root/'manifest.json').read_text())
    hashes={e['id']:e['sha256'] for e in m['tasks']}
    for file in (ROOT/'results/rack_v0.2').rglob('*.json'):
        result=json.loads(file.read_text())
        if 'scene_sha256' in result:
            assert result['scene_sha256']==hashes[result.get('task_id',result.get('scene_id'))],file
    # gosuite suites: task and witness hashes; every recorded run must point at the current task bytes.
    counts={}
    for version in ['v0.3','v0.4']:
        suite=ROOT/'data'/f'suite_{version}';m=json.loads((suite/'manifest.json').read_text());n_tasks=n_runs=0
        for entry in m['tasks']:
            assert sha(suite/entry['file'])==entry['sha256'],entry['id']
            assert sha(suite/entry['witness_file'])==entry['witness_sha256'],entry['id']
            if 'witness_nominal_file' in entry:
                assert sha(suite/entry['witness_nominal_file'])==entry['witness_nominal_sha256'],entry['id']
            n_tasks+=1
        hashes={e['id']:e['sha256'] for e in m['tasks']}
        for file in (ROOT/'results'/f'suite_{version}').glob('*/runs/*/*.json'):
            record=json.loads(file.read_text())
            assert record['task_sha256']==hashes[record['task_id']],file;n_runs+=1
        counts[version]=(n_tasks,n_runs)
    # every results table's goal-feasible column must match its episodes and the SAME suite's validation flags
    # (task ids repeat across suite versions, so flags from another suite would still look plausible)
    n_tables=0
    for version in ['v0.3','v0.4']:
        res=ROOT/'results'/f'suite_{version}';flags=json.loads((res/'validation'/'goal_feasibility.json').read_text())
        for table in sorted(res.glob('*/runs.csv')):
            with table.open(newline='') as fh:
                column={(r['label'],r['task_id'],r['seed']):r['feasible_episode_success'] for r in csv.DictReader(fh)}
            for file in (table.parent/'runs').glob('*/*.json'):
                rec=json.loads(file.read_text());ex=rec.get('execution') or {}
                mask=[c=='0' for c in flags[rec['task_id']][:ex.get('episodes') or len(flags[rec['task_id']])]]
                want=None if not any(mask) else (sum(s=='1' for s,m in zip(ex['episode_success'],mask) if m)/sum(mask) if ex else 0.)
                got=column[(rec['label'],rec['task_id'],str(rec['seed']))]
                assert (got=='' if want is None else got!='' and abs(float(got)-want)<1e-9),(file,got,want)
            n_tables+=1
    print(f'Verified {asset_count} asset hashes, {task_count} legacy tasks, '
          + ', '.join(f'{v}: {a} tasks and {b} run records' for v,(a,b) in counts.items())
          + f', the goal-feasible column of {n_tables} results tables, and recorded rack source hashes.')

if __name__=='__main__':main()
