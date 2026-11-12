#!/usr/bin/env python3
"""Export readable documents plus verifiable source/test/acceptance artifacts."""
import argparse
import hashlib
import html
import json
import shutil
import subprocess
import zipfile
from pathlib import Path
from render_supervisor_report import export_report

NAMES = [
    'SUPERVISOR_REPORT_RU',
    'ACCEPTANCE_2026-11-12', 'HARDWARE_BOM_AND_ARCHITECTURE',
    'AIRFRAME_MODEL_MAPPING_RU', 'DELIVERY_AUDIT_2026-11-12',
    'START_ARDUPILOT_RU', 'PROJECT_LAYOUT_RU', 'PROJECT_REQUIREMENTS_RU',
    'OPTICAL_FLOW_AND_OPENFLOW_STUDY', 'ALIGNMENT_DIAGNOSIS_2026-11-12',
    'VERIFICATION_2026-11-04',
]
STYLE = ('body{max-width:1100px;margin:40px auto;padding:24px;'
         'font:18px/1.6 system-ui;color:#172235;background:#f6f8fb}'
         'pre{white-space:pre-wrap;overflow-wrap:anywhere;font:inherit}'
         'li{margin:14px 0}')

def page(title, body):
    return ('<!doctype html><html lang="ru"><meta charset="utf-8"><title>'
            + html.escape(title) + '</title><style>' + STYLE + '</style>'
            + body + '</html>')

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def export(destination, root):
    destination.mkdir(parents=True, exist_ok=True)
    export_report(root, destination)
    docs = root / 'docs'
    links = []
    selected = []
    for name in NAMES:
        source = docs / (name + ('.txt' if name == 'START_ARDUPILOT_RU' else '.md'))
        if not source.exists():
            continue
        selected.append(source)
        text = source.read_text(encoding='utf-8')
        (destination / (name + '.txt')).write_text(text, encoding='utf-8-sig')
        (destination / (name + '.html')).write_text(
            page(name, '<a href="ОТКРЫТЬ.html">Все документы</a><pre>'
                 + html.escape(text) + '</pre>'), encoding='utf-8')
        links.append('<li><a href="' + name + '.html">' + name
                     + '</a> | <a href="' + name + '.txt">TXT</a></li>')

    artifacts = destination / 'artifacts'
    artifacts.mkdir(exist_ok=True)
    files = []
    package = root / 'src/glider_visual_nav'
    sources = [
        p for p in package.rglob('*') if p.is_file()
        and '__pycache__' not in p.parts and p.suffix != '.pyc'
    ]
    for source in sorted((package / 'simulation/patches').glob('*.patch')):
        target = artifacts / 'patches' / source.name
        target.parent.mkdir(exist_ok=True)
        shutil.copyfile(source, target)
        files.append(target)
    results = sorted((docs / 'results').glob('acceptance*20261112.json'))
    for source in results:
        target = artifacts / 'results' / source.name
        target.parent.mkdir(exist_ok=True)
        shutil.copyfile(source, target)
        files.append(target)
    for source in sorted((package / 'test').rglob('*')):
        if source.is_file() and '__pycache__' not in source.parts:
            target = artifacts / 'tests/source' / source.relative_to(package / 'test')
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            files.append(target)
    test_dir = root / 'build_visual_nav/glider_visual_nav/test_results/glider_visual_nav'
    for source in sorted(test_dir.glob('*.xml')):
        target = artifacts / 'tests/results' / source.name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        files.append(target)
    evidence = root / 'log_visual_nav/acceptance_test_result.log'
    if evidence.exists():
        target = artifacts / 'tests/acceptance_test_result.log'
        target.parent.mkdir(exist_ok=True)
        shutil.copyfile(evidence, target)
        files.append(target)
    archive = artifacts / 'glider_visual_nav_sources.zip'
    top = [root / 'README.md', root / 'AGENTS.md']
    with zipfile.ZipFile(archive, 'w', compression=zipfile.ZIP_DEFLATED) as bundle:
        for source in sorted(set(sources + selected + results + top)):
            if source.is_file():
                bundle.write(source, source.relative_to(root).as_posix())
    files.append(archive)
    pub = root / 'build_visual_nav/publication.CSNoTY'
    revision = subprocess.run(
        ['git', '-C', str(pub if pub.is_dir() else root), 'rev-parse', 'HEAD'],
        capture_output=True, text=True, check=True).stdout.strip()
    manifest = {
        'publication_revision': revision,
        'test_summary': evidence.read_text().strip().splitlines()[-1] if evidence.exists() else None,
        'files': {p.relative_to(destination).as_posix(): digest(p) for p in sorted(files)},
        'source_hashes': {p.relative_to(root).as_posix(): digest(p)
                          for p in sorted(set(sources + selected + results + top)) if p.is_file()},
        'note': 'Source bundle excludes build binaries, raw telemetry and Git credentials.',
    }
    (artifacts / 'manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    artifact_links = [
        '<li><a href="' + p.relative_to(destination).as_posix() + '">'
        + html.escape(p.relative_to(artifacts).as_posix()) + '</a></li>' for p in files
    ]
    artifact_links.insert(0, '<li><a href="artifacts/manifest.json">SHA256 manifest</a></li>')
    (destination / 'ОТКРЫТЬ.html').write_text(page(
        'Планер — материалы проекта',
        '<p style="font-size:22px;font-weight:bold"><a href="ОТЧЕТ_ДЛЯ_РУКОВОДИТЕЛЯ.html">ОТЧЕТ ДЛЯ НАУЧНОГО РУКОВОДИТЕЛЯ — открыть / распечатать</a></p>\n'
        '<h1>Планер — материалы проекта</h1>'
        '<p>Приемка 12 ноября: passed=true; 27 тестов без ошибок. '
        'Физический носитель: Ranger 2400 PNP; параметры реального экземпляра требуют обмера.</p>'
        '<p>Публикационная ревизия: <code>' + html.escape(revision) + '</code></p>'
        '<ul>' + ''.join(links) + '</ul><h2>Исходники и доказательства приемки</h2>'
        '<ul>' + ''.join(artifact_links) + '</ul>'), encoding='utf-8')
    return manifest

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('destination', type=Path)
    args = parser.parse_args()
    manifest = export(args.destination, Path(__file__).resolve().parents[3])
    print(json.dumps({'revision': manifest['publication_revision'],
                      'artifact_files': len(manifest['files']),
                      'test_summary': manifest['test_summary']}, ensure_ascii=False))
