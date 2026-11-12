"""Content-identical visual resource namespaces for Gazebo OGRE incarnations.

OGRE caches mesh material resources by URI. Deleting the last instance can leave
that cached mesh without its texture; a later create must reload its own URI.
Only visual mesh URI text changes. Physics geometry and online inputs do not.
"""
import hashlib
import json
from pathlib import Path
import shutil
import uuid
import xml.etree.ElementTree as ET


def isolate_visual_assets(sdf,*,asset_root,models_root):
    if not asset_root:return sdf
    root=ET.fromstring(sdf);sources={}
    models=Path(models_root).resolve()
    for uri in root.findall('.//visual/geometry/mesh/uri'):
        if not uri.text or not uri.text.startswith('model://'):
            raise ValueError('CANONICAL_MODEL_VISUAL_URI_REQUIRED')
        relative=Path(uri.text[len('model://'):])
        source=(models/relative).resolve()
        if relative.is_absolute() or not source.is_relative_to(models) or not source.is_file():
            raise ValueError('VISUAL_RESOURCE_UNAVAILABLE')
        package=relative.parts[0]
        sources.setdefault(package,[]).append((uri,relative,source))
    if not sources:return sdf
    folder=Path(asset_root)/uuid.uuid4().hex;folder.mkdir(parents=True,exist_ok=False)
    receipt=[]
    for package,entries in sources.items():
        original=models/package;copied=folder/package
        shutil.copytree(original,copied)
        files=[]
        for source in sorted(original.rglob('*')):
            if not source.is_file():continue
            relative=source.relative_to(original);target=copied/relative
            digest=hashlib.sha256(source.read_bytes()).hexdigest()
            if hashlib.sha256(target.read_bytes()).hexdigest()!=digest:
                raise ValueError('VISUAL_RESOURCE_COPY_MISMATCH')
            files.append(dict(relative=str(relative),sha256=digest,bytes=source.stat().st_size))
        for uri,relative,source in entries:
            previous=uri.text;uri.text=(folder/relative).resolve().as_uri()
            receipt.append(dict(source_uri=previous,instance_uri=uri.text,files=files))
    (folder/'manifest.json').write_text(json.dumps(dict(scope='visual_uri_only_content_identical',resources=receipt),indent=2)+'\n')
    return ET.tostring(root,encoding='unicode')
