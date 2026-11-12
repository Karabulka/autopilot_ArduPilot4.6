#!/usr/bin/env python3
"""Two-page, print-ready HTML from the versioned supervisor Markdown report."""
import html
import re
from pathlib import Path

STYLE = """
@page { size: A4; margin: 12mm; }
* { box-sizing: border-box; }
body { margin: 0; font: 10pt/1.25 Arial, sans-serif; color: #152336; }
.sheet { break-after: page; page-break-after: always; }
.sheet:last-child { break-after: auto; page-break-after: auto; }
h1 { font-size: 16pt; line-height: 1.12; margin: 0 0 3mm; color: #123c60; }
h2 { font-size: 11.5pt; margin: 3.5mm 0 2mm; color: #123c60; }
p { margin: 0 0 2.2mm; orphans: 2; widows: 2; }
table { width: 100%; border-collapse: collapse; margin: 2mm 0 3mm; font-size: 9pt; line-height: 1.18; }
th { background: #e9f0f6; text-align: left; }
th,td { padding: 1.5mm 2mm; border: .2mm solid #bdcad7; vertical-align: top; }
tr { break-inside: avoid; page-break-inside: avoid; }
thead { display: table-header-group; }
pre { margin: 2mm 0 3mm; padding: 2.5mm; background: #edf2f6; font: 8.2pt/1.5 Consolas, monospace; white-space: pre-wrap; }
a { color: #164e76; text-decoration: none; }
.footer { border-top: .2mm solid #c8d2de; padding-top: 2mm; margin-top: 3mm; font-size: 8pt; color: #526477; }
@media screen {
 body { background: #dfe5ec; }
 .sheet { width: 210mm; min-height: 297mm; margin: 8mm auto; padding: 12mm; background: white; box-shadow: 0 2px 12px #aab4c0; }
}
"""

def inline(text):
    escaped = html.escape(text)
    escaped = re.sub(r'\[([^\]]+)\]\((https?://[^)]+)\)',
                     r'<a href="\2">\1</a>', escaped)
    return re.sub(r'\*\*([^*]+)\*\*', r'<strong>\1</strong>', escaped)

def render_section(source):
    out, paragraph = [], []
    table, code = False, None
    def flush():
        if paragraph:
            out.append('<p>' + inline(' '.join(paragraph)) + '</p>')
            paragraph.clear()
    for line in source.splitlines():
        if line.startswith('~~~'):
            flush()
            if code is None:
                code = []
            else:
                out.append('<pre>' + html.escape('\n'.join(code)) + '</pre>')
                code = None
            continue
        if code is not None:
            code.append(line)
            continue
        if table and not line.startswith('|'):
            out.append('</tbody></table>')
            table = False
        if line.startswith('|'):
            flush()
            cells = [c.strip() for c in line.strip().strip('|').split('|')]
            if all(re.fullmatch(r'[: -]+', c) for c in cells):
                continue
            if not table:
                out.append('<table><thead><tr>' + ''.join('<th>'+inline(c)+'</th>' for c in cells) + '</tr></thead><tbody>')
                table = True
            else:
                out.append('<tr>' + ''.join('<td>'+inline(c)+'</td>' for c in cells) + '</tr>')
        elif line.startswith('#'):
            flush()
            marker, title = line.split(' ', 1)
            level = len(marker)
            out.append(f'<h{level}>' + inline(title) + f'</h{level}>')
        elif line.strip():
            paragraph.append(line)
        else:
            flush()
    flush()
    if table:
        out.append('</tbody></table>')
    return '\n'.join(out)

def export_report(root, destination):
    source = root / 'docs/SUPERVISOR_REPORT_RU.md'
    text = source.read_text(encoding='utf-8')
    sections = text.split('<!-- PAGE_BREAK -->')
    if len(sections) != 2:
        raise ValueError('Supervisor report must define exactly two print sections')
    bodies = [render_section(section) for section in sections]
    content = '\n'.join(
        '<section class="sheet">' + body +
        f'<div class="footer">Статус проекта · 12 ноября 2026 · {n}/2</div></section>'
        for n, body in enumerate(bodies, 1))
    document = ('<!doctype html>\n<html lang="ru"><head><meta charset="utf-8">'
                '<meta name="viewport" content="width=device-width,initial-scale=1">'
                '<title>Отчет для научного руководителя</title><style>' + STYLE +
                '</style></head><body>' + content + '</body></html>')
    (destination / 'ОТЧЕТ_ДЛЯ_РУКОВОДИТЕЛЯ.html').write_text(document, encoding='utf-8')
    plain = text.replace('<!-- PAGE_BREAK -->', '\n')
    plain = plain.replace('~~~bash', '').replace('~~~', '').replace('**', '')
    (destination / 'ОТЧЕТ_ДЛЯ_РУКОВОДИТЕЛЯ.txt').write_text(plain, encoding='utf-8-sig')
    return document
