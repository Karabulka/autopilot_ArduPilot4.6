#!/usr/bin/env python3
"""Build the two-page supervisor PDF from the versioned Markdown source."""
import argparse
import html
import re
from pathlib import Path
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT, TA_CENTER
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Preformatted

def inline(text):
    text = html.escape(text)
    text = re.sub(r'\[([^\]]+)\]\((https?://[^)]+)\)', r'<a href="\2" color="#000000">\1</a>', text)
    return text.replace('**', '')

def build(source, output, font_dir):
    regular = font_dir / 'times.ttf'
    bold = font_dir / 'timesbd.ttf'
    if not regular.exists():
        regular = font_dir / 'DejaVuSerif.ttf'
        bold = font_dir / 'DejaVuSerif-Bold.ttf'
    pdfmetrics.registerFont(TTFont('Report', str(regular)))
    pdfmetrics.registerFont(TTFont('ReportBold', str(bold)))
    pdfmetrics.registerFontFamily('Report', normal='Report', bold='ReportBold')
    styles = {
        'body': ParagraphStyle('body', fontName='Report', fontSize=10.5, leading=12.5, spaceAfter=5),
        'h1': ParagraphStyle('h1', fontName='ReportBold', fontSize=14, leading=16, alignment=TA_CENTER, textColor=colors.black, spaceAfter=9),
        'h2': ParagraphStyle('h2', fontName='ReportBold', fontSize=11, leading=13, textColor=colors.black, spaceBefore=7, spaceAfter=5),
        'cell': ParagraphStyle('cell', fontName='Report', fontSize=10.3, leading=11.6),
        'code': ParagraphStyle('code', fontName='Report', fontSize=8.2, leading=12, borderPadding=0),
    }
    story = []
    width = 174 * mm
    def para(text, style='body'):
        return Paragraph(inline(text), styles[style])
    def add_table(rows):
        columns = len(rows[0])
        widths = [43*mm,131*mm] if columns==2 else [65*mm,58*mm,51*mm]
        if columns==3 and 'Задача' in rows[0][0]:
            widths=[39*mm,38*mm,97*mm]
        if columns==3 and 'Этап' in rows[0][0]:
            widths=[31*mm,89*mm,54*mm]
        cells=[[para(c, 'cell') for c in row] for row in rows]
        table=Table(cells,colWidths=widths,repeatRows=1,hAlign='LEFT')
        table.setStyle(TableStyle([

            ('GRID',(0,0),(-1,-1),.35,colors.HexColor('#555555')),
            ('VALIGN',(0,0),(-1,-1),'TOP'),
            ('LEFTPADDING',(0,0),(-1,-1),5),('RIGHTPADDING',(0,0),(-1,-1),5),
            ('TOPPADDING',(0,0),(-1,-1),3),('BOTTOMPADDING',(0,0),(-1,-1),3),
        ]))
        story.extend([table,Spacer(1,5)])
    blocks=source.read_text(encoding='utf-8').split('<!-- PAGE_BREAK -->')
    if len(blocks)!=2:raise ValueError('Expected two report pages')
    for page_number,block in enumerate(blocks):
        paragraph=[];rows=[];code=None
        def flush():
            if paragraph:
                story.append(para(' '.join(paragraph)));paragraph.clear()
        for line in block.splitlines():
            if line.startswith('~~~'):
                flush()
                if code is None:code=[]
                else:
                    story.extend([Spacer(1,4),Preformatted('\n'.join(code),styles['code']),Spacer(1,7)])
                    code=None
                continue
            if code is not None:code.append(line);continue
            if rows and not line.startswith('|'):add_table(rows);rows=[]
            if line.startswith('|'):
                flush()
                cells=[c.strip() for c in line.strip().strip('|').split('|')]
                if not all(re.fullmatch(r'[: -]+',c) for c in cells):rows.append(cells)
            elif line.startswith('#'):
                flush();mark,title=line.split(' ',1);story.append(para(title,'h1' if len(mark)==1 else 'h2'))
            elif line.strip():paragraph.append(line)
            else:flush()
        flush()
        if rows:add_table(rows)
        if page_number==0:story.append(PageBreak())
    output.parent.mkdir(parents=True,exist_ok=True)
    def footer(canvas,doc):
        canvas.setFont('Report',10)
        canvas.setFillColor(colors.black)
        canvas.drawCentredString(105*mm,9*mm,str(doc.page))
    doc=SimpleDocTemplate(str(output),pagesize=(210*mm,297*mm),rightMargin=18*mm,leftMargin=18*mm,
                          topMargin=15*mm,bottomMargin=15*mm,title='Навигация планера без GPS: статус проекта',
                          author='Проект glider_visual_nav')
    doc.build(story,onFirstPage=footer,onLaterPages=footer)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--font-dir',type=Path,required=True);a=p.parse_args();build(a.source,a.output,a.font_dir)
