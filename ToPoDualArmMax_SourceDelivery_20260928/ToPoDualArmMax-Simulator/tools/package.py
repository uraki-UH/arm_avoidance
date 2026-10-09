"""実行データを除いたソース配布ZIPとSHA-256マニフェストの生成。"""
from pathlib import Path
import argparse,hashlib,json,zipfile

ROOT=Path(__file__).resolve().parents[1]
EXCLUDED={'runtime','exports','node_modules','__pycache__','.git','live'}
TOP={'src','package-lock.json','tsconfig.json','vite.config.ts','app','tools','tests','docs','licenses','integrations','package.json','README.md','LICENSE.md','THIRD_PARTY_NOTICES.md','.gitignore','START.cmd','Start-Simulator.ps1','Stop-Simulator.ps1','start.sh'}
def files():
    for p in sorted(ROOT.rglob('*')):
        rel=p.relative_to(ROOT)
        if rel.parts[0] not in TOP or any(x in EXCLUDED for x in rel.parts):continue
        if p.name.startswith('.env') or p.suffix.lower() in {'.log','.pyc','.zip','.pem','.key'}:continue
        if p.is_symlink():raise ValueError('Symlink not allowed in delivery: '+str(rel))
        if p.is_file():yield p
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('output',type=Path);args=parser.parse_args()
    listed=list(files())
    manifest={'format':'topo-source-manifest/1','version':json.loads((ROOT/'package.json').read_text(encoding='utf-8'))['version'],
              'self_excluded':'MANIFEST.json is excluded from its own hash list; use the ZIP SHA-256 for whole-archive verification.',
              'files':[{'path':p.relative_to(ROOT).as_posix(),'bytes':p.stat().st_size,'sha256':digest(p)} for p in listed]}
    (ROOT/'MANIFEST.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(args.output,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in listed+[ROOT/'MANIFEST.json']:
            info=zipfile.ZipInfo.from_file(p,'ToPoDualArmMax-Simulator/'+p.relative_to(ROOT).as_posix())
            info.compress_type=zipfile.ZIP_DEFLATED
            if p.suffix=='.sh':info.external_attr=0o100755<<16
            z.writestr(info,p.read_bytes())
    with zipfile.ZipFile(args.output) as z:
        bad=z.testzip()
        if bad:raise ValueError('ZIP CRC failure: '+bad)
    args.output.with_suffix('.zip.sha256').write_text(digest(args.output)+'  '+args.output.name+'\n',encoding='ascii')
    print(json.dumps({'files':len(listed)+1,'bytes':args.output.stat().st_size,'sha256':digest(args.output)}))
if __name__=='__main__':main()
