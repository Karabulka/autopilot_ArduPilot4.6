#!/usr/bin/env python3
"""Build the two-page supervisor PDF from the versioned Markdown source."""
import argparse
import html
import re
from pathlib import Path
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Preformatted

def inline(text):
    text = html.escape(text)
    text = re.sub(r'\[([^\]]+)\]\((https?://[^)]+)\)', r'<a href="\2" color="#164e76">\1</a>', text)
    return re.sub(r'\*\*([^*]+)\*\*', r'<b>\1</b>', text)

def build(source, output, font_dir):
    regular = font_dir / 'arial.ttf'
    bold = font_dir / 'arialbd.ttf'
    if not regular.exists():
        regular = font_dir / 'DejaVuSans.ttf'
        bold = font_dir / 'DejaVuSans-Bold.ttf'
    pdfmetrics.registerFont(TTFont('Report', str(regular)))
    pdfmetrics.registerFont(TTFont('ReportBold', str(bold)))
    pdfmetrics.registerFontFamily('Report', normal='Report', bold='ReportBold')
    styles = {
        'body': ParagraphStyle('body', fontName='Report', fontSize=9.3, leading=11.5, spaceAfter=5),
        'h1': ParagraphStyle('h1', fontName='ReportBold', fontSize=17, leading=20, textColor=colors.HexColor('#123c60'), spaceAfter=7),
        'h2': ParagraphStyle('h2', fontName='ReportBold', fontSize=11.2, leading=14, textColor=colors.HexColor('#123c60'), spaceBefore=7, spaceAfter=5),
        'cell': ParagraphStyle('cell', fontName='Report', fontSize=9, leading=11),
        'code': ParagraphStyle('code', fontName='Report', fontSize=8.2, leading=12, backColor=colors.HexColor('#edf2f6'), borderPadding=6),
    }
    story = []
    width = 186 * mm
    def para(text, style='body'):
        return Paragraph(inline(text), styles[style])
    def add_table(rows):
        columns = len(rows[0])
        widths = [48*mm,138*mm] if columns==2 else [59*mm,63*mm,64*mm]
        if columns==3 and 'Задача' in rows[0][0]:
            widths=[42*mm,40*mm,104*mm]
        if columns==3 and 'Этап' in rows[0][0]:
            widths=[32*mm,95*mm,59*mm]
        cells=[[para(c, 'cell') for c in row] for row in rows]
        table=Table(cells,colWidths=widths,repeatRows=1,hAlign='LEFT')
        table.setStyle(TableStyle([
            ('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e7eff6')),
            ('GRID',(0,0),(-1,-1),.35,colors.HexColor('#bac9d7')),
            ('VALIGN',(0,0),(-1,-1),'TOP'),
            ('LEFTPADDING',(0,0),(-1,-1),6),('RIGHTPADDING',(0,0),(-1,-1),6),
            ('TOPPADDING',(0,0),(-1,-1),4),('BOTTOMPADDING',(0,0),(-1,-1),4),
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
        canvas.setStrokeColor(colors.HexColor('#bbc9d6'));canvas.line(12*mm,11*mm,198*mm,11*mm)
        canvas.setFont('Report',8);canvas.setFillColor(colors.HexColor('#526477'))
        canvas.drawString(12*mm,7*mm,'Статус проекта | 12 ноября 2026')
        canvas.drawRightString(198*mm,7*mm,str(doc.page))
    doc=SimpleDocTemplate(str(output),pagesize=(210*mm,297*mm),rightMargin=12*mm,leftMargin=12*mm,
                          topMargin=12*mm,bottomMargin=15*mm,title='Навигация планера без GPS: статус проекта',
                          author='Проект glider_visual_nav')
    doc.build(story,onFirstPage=footer,onLaterPages=footer)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--font-dir',type=Path,required=True);a=p.parse_args();build(a.source,a.output,a.font_dir)
