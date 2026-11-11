"""Check bundled asset/task hashes and confirm recorded rack results match inputs."""
import hashlib,json,xml.etree.ElementTree as ET
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
    # v0.3 suite: task and witness hashes, and every recorded run must point at the current task bytes.
    suite=ROOT/'data/suite_v0.3';m=json.loads((suite/'manifest.json').read_text());suite_count=run_count=0
    for entry in m['tasks']:
        assert sha(suite/entry['file'])==entry['sha256'],entry['id']
        assert sha(suite/entry['witness_file'])==entry['witness_sha256'],entry['id'];suite_count+=1
    hashes={e['id']:e['sha256'] for e in m['tasks']}
    for file in (ROOT/'results/suite_v0.3').glob('*/runs/*/*.json'):
        record=json.loads(file.read_text())
        assert record['task_sha256']==hashes[record['task_id']],file;run_count+=1
    print(f'Verified {asset_count} asset hashes, {task_count} legacy tasks, {suite_count} v0.3 tasks, '
          f'{run_count} v0.3 run records and recorded rack source hashes.')

if __name__=='__main__':main()
