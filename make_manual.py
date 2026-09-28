#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 markdown 学习笔记转成排版好的 HTML + PDF。

用法:
    python make_notes.py

流水线（跟「ROS数据流向」那套一致）:
    markdown-it-py  →  HTML  →  Edge 无头渲染  →  PyMuPDF 补页脚+压缩

为什么不用 WeasyPrint：这台机器上它 import 就崩（缺 Pango/GTK）。
"""
import os
import shutil
import subprocess
import sys
import tempfile

from markdown_it import MarkdownIt
import fitz  # PyMuPDF

HERE = os.path.dirname(os.path.abspath(__file__))
MD = os.path.join(HERE, "使用手册.md")
HTML = os.path.join(HERE, "使用手册.html")
PDF = os.path.join(HERE, "使用手册.pdf")

EDGE = [
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
]
FONT = r"C:\Windows\Fonts\simhei.ttf"

CSS = """
:root{--ink:#0f172a;--muted:#64748b;--line:#e2e8f0;--accent:#2563eb;
      --code-bg:#0f172a;--warn:#f59e0b;--ok:#059669;}
*{box-sizing:border-box}
body{margin:0;padding:0 0 60px;background:#f8fafc;color:var(--ink);
     font:15px/1.8 -apple-system,"Segoe UI","Microsoft YaHei",sans-serif;}
.wrap{max-width:900px;margin:0 auto;padding:0 26px;background:#fff;
      border-left:1px solid var(--line);border-right:1px solid var(--line);
      min-height:100vh;box-shadow:0 0 40px rgba(0,0,0,.04);}
h1{font-size:27px;margin:0 0 8px;padding:34px 0 0;letter-spacing:.3px;}
h1:first-child{border-bottom:3px solid var(--ink);padding-bottom:16px;margin-bottom:26px;}
h2{font-size:21px;margin:44px 0 16px;padding:10px 0 10px 14px;
   border-left:5px solid var(--accent);background:#f8fafc;border-radius:0 8px 8px 0;}
h3{font-size:17px;margin:30px 0 12px;color:#334155;
   border-bottom:1px solid var(--line);padding-bottom:7px;}
h4{font-size:15.5px;margin:22px 0 10px;color:#475569;}
p{margin:12px 0;}
a{color:var(--accent);}
code{background:#f1f5f9;padding:2px 6px;border-radius:4px;color:#be123c;
     font:13px/1.5 "Cascadia Mono",Consolas,"Courier New",monospace;}
pre{background:var(--code-bg);color:#e2e8f0;padding:15px 18px;border-radius:9px;
    overflow-x:auto;font:13px/1.65 "Cascadia Mono",Consolas,monospace;margin:14px 0;}
pre code{background:none;color:inherit;padding:0;font-size:inherit;}
blockquote{margin:16px 0;padding:12px 18px;background:#eff6ff;border-radius:0 8px 8px 0;
           border-left:4px solid var(--accent);color:#1e293b;}
blockquote p{margin:6px 0;}
blockquote strong{color:#0f172a;}
table{width:100%;border-collapse:collapse;margin:16px 0;font-size:13.5px;}
th{background:#f1f5f9;text-align:left;padding:9px 11px;border:1px solid var(--line);
   font-weight:600;color:#334155;}
td{padding:9px 11px;border:1px solid var(--line);vertical-align:top;}
tr:nth-child(even) td{background:#fcfdfe;}
ul,ol{padding-left:26px;margin:12px 0;}
li{margin:5px 0;}
hr{border:none;border-top:2px dashed var(--line);margin:40px 0;}
.wrap > hr + h1{margin-top:0;}

@page{size:A4;margin:14mm 12mm}
@media print{
  body{background:#fff;font-size:12.5px;padding-bottom:0;}
  .wrap{max-width:none;padding:0;border:none;box-shadow:none;}
  h1{font-size:21px;padding-top:0;}
  h2{font-size:17px;margin:26px 0 12px;break-after:avoid;page-break-after:avoid;}
  h3{font-size:15px;break-after:avoid;page-break-after:avoid;}
  pre,blockquote,th,td{ -webkit-print-color-adjust:exact;print-color-adjust:exact;}
  /* ⚠️ pre 不能加 break-inside:avoid —— 长代码块一页放不下时，
     整块会被推到下一页，上一页留一大片空白（踩过）。
     代码块就该允许跨页断开。*/
  pre{font-size:10.5px;padding:11px 13px;line-height:1.5;
      orphans:3;widows:3;}
  code{font-size:11px;}
  table{font-size:11.5px;} th,td{padding:6px 8px;}
  tr,blockquote,li{break-inside:avoid;page-break-inside:avoid;}
}
"""


def md_to_html():
    md = MarkdownIt("default").enable("table")
    body = md.render(open(MD, encoding="utf-8").read())
    html = (
        '<!DOCTYPE html>\n<html lang="zh-CN">\n<head>\n'
        '<meta charset="utf-8">\n'
        '<title>从零手写四车协同 · 学习笔记</title>\n'
        '<style>%s</style>\n</head>\n<body>\n<div class="wrap">\n%s\n</div>\n'
        '</body>\n</html>\n' % (CSS, body)
    )
    open(HTML, "w", encoding="utf-8").write(html)
    return len(html)


def html_to_pdf():
    edge = next((p for p in EDGE if os.path.exists(p)), None)
    if not edge:
        sys.exit("找不到 msedge.exe")
    tmp = tempfile.mkdtemp(prefix="edge_pdf_")
    try:
        subprocess.run(
            [edge, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
             "--user-data-dir=" + tmp, "--print-to-pdf=" + PDF,
             "file:///" + HTML.replace("\\", "/")],
            check=True, timeout=240,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def add_footer_and_compress():
    doc = fitz.open(PDF)
    total = doc.page_count
    for i, page in enumerate(doc):
        r = page.rect
        page.insert_text((r.width / 2 - 26, r.height - 16),
                         "第 %d / %d 页" % (i + 1, total),
                         fontsize=9, fontname="hei", fontfile=FONT,
                         color=(0.55, 0.58, 0.62))
    doc.save(PDF + ".tmp", garbage=4, deflate=True, clean=True)
    doc.close()
    os.replace(PDF + ".tmp", PDF)
    return total


if __name__ == "__main__":
    if not os.path.exists(MD):
        sys.exit("找不到 %s" % MD)
    print("1/3  markdown -> html ...")
    size = md_to_html()
    print("     写了 %d 字节" % size)
    print("2/3  Edge 渲染 pdf ...")
    html_to_pdf()
    print("3/3  补页脚 + 压缩 ...")
    n = add_footer_and_compress()
    print()
    print("完成: %s" % PDF)
    print("      %d 页, %.1f MB" % (n, os.path.getsize(PDF) / 1048576))
