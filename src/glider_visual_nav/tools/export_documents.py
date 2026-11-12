#!/usr/bin/env python3
"""Export only project documents to standalone readable HTML/TXT."""
import argparse,html
from pathlib import Path
NAMES=['ACCEPTANCE_2026-11-12','START_ARDUPILOT_RU','PROJECT_LAYOUT_RU',
       'PROJECT_REQUIREMENTS_RU','HARDWARE_BOM_AND_ARCHITECTURE',
       'OPTICAL_FLOW_AND_OPENFLOW_STUDY','ALIGNMENT_DIAGNOSIS_2026-11-12',
       'VERIFICATION_2026-11-04']
STYLE='body{max-width:1100px;margin:40px auto;padding:24px;font:18px/1.6 system-ui;color:#172235;background:#f6f8fb}pre{white-space:pre-wrap;overflow-wrap:anywhere;font:inherit}li{margin:14px 0}'
def page(title,body):
    return '<!doctype html><html lang="ru"><meta charset="utf-8"><title>'+title+'</title><style>'+STYLE+'</style>'+body+'</html>'
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('destination',type=Path);a=p.parse_args()
    docs=Path(__file__).resolve().parents[3]/'docs';a.destination.mkdir(parents=True,exist_ok=True);links=[]
    for name in NAMES:
        source=docs/(name+('.txt' if name=='START_ARDUPILOT_RU' else '.md'))
        if not source.exists():continue
        text=source.read_text()
        (a.destination/(name+'.txt')).write_text(text,encoding='utf-8-sig')
        (a.destination/(name+'.html')).write_text(page(name,'<a href="ОТКРЫТЬ.html">Все документы</a><pre>'+html.escape(text)+'</pre>'))
        links.append('<li><a href="'+name+'.html">'+name+'</a> | <a href="'+name+'.txt">TXT</a></li>')
    (a.destination/'ОТКРЫТЬ.html').write_text(page('Планер — материалы проекта',
        '<h1>Планер — материалы проекта</h1><p>Актуальные измерения — в ACCEPTANCE_2026-11-12. Старые отчеты сохранены как история диагностики.</p><ul>'+''.join(links)+'</ul>'))
