"""Bundle only v0.2 rack scenes, code, evidence and manufacturer models."""
from pathlib import Path
import hashlib,json,zipfile
ROOT=Path(__file__).resolve().parents[1]

def main():
    files=[]
    for folder in ['assets/ur5e_robotiq','gobench','rackbench','data/rack_v0.2','results/rack_v0.2','previews/rack_v0.2']:
        files.extend(p for p in (ROOT/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    files.extend(ROOT/name for name in ['RACK_BENCHMARK.zh-CN.md','RACK_RESULTS.zh-CN.md','requirements.txt','requirements-lock.txt',
        'requirements-assets.txt','tools/python.sh','tools/prepare_rack_robot.py','tools/package_rack_release.py','tests/test_rackbench.py'])
    files=sorted(set(files))
    manifest=dict(version='0.2.0',scope='3 development warehouse rack geometries, 6 directional tasks',
        files={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    mf=ROOT/'rack_release_manifest.json';mf.write_text(json.dumps(manifest,indent=2)+'\n');files.append(mf)
    dest=ROOT/'warehouse_rack_v0.2.zip'
    with zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED) as z:
        for f in files:z.write(f,'warehouse_rack_v0.2/'+str(f.relative_to(ROOT)))
        z.write(ROOT/'RACK_BENCHMARK.zh-CN.md','warehouse_rack_v0.2/README.zh-CN.md')
    with zipfile.ZipFile(dest) as z:
        assert z.testzip() is None
        for name,digest in manifest['files'].items():assert hashlib.sha256(z.read('warehouse_rack_v0.2/'+name)).hexdigest()==digest
    print(json.dumps(dict(path=str(dest),files=len(files)+1,bytes=dest.stat().st_size,sha256=hashlib.sha256(dest.read_bytes()).hexdigest()),indent=2))

if __name__=='__main__':main()
