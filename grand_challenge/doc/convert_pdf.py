import os
import pdfkit

# Define the directory containing HTML files
html_directory_main = 'D:/Documents/UMD_Docs/Grad-school/ENPM701/Grand-Challenge/grand_challenge_submission/doc/grand_challenge_submission'
html_directory_racer = 'D:/Documents/UMD_Docs/Grad-school/ENPM701/Grand-Challenge/grand_challenge_submission/doc/grand_challenge_submission/racer'
html_directory_Perception = 'D:/Documents/UMD_Docs/Grad-school/ENPM701/Grand-Challenge/grand_challenge_submission/doc/grand_challenge_submission/Perception'

#pdfkit.from_file(os.path.join(html_directory_main, html_file), 'output.pdf')

html = [html_directory_main,html_directory_racer,html_directory_Perception]
# List HTML files in the directory
html_files = [f for f in os.listdir(html[0]) if f.endswith('.html')]
html_f_2 = [f for f in os.listdir(html[1]) if f.endswith('.html')]
html_f_3 = [f for f in os.listdir(html[2]) if f.endswith('.html')]
i = 0

pdf_files = []

# Convert each HTML file to PDF
for html_file in html_files:
    pdf_file = os.path.join(html_directory_main, html_file.replace('.html', '.pdf'))
    pdfkit.from_file(os.path.join(html_directory_main, html_file), pdf_file)
    
for html_file in html_f_2:
    pdf_file = os.path.join(html_directory_main, html_file.replace('.html', '.pdf'))
    pdfkit.from_file(os.path.join(html_directory_main, html_file), pdf_file)
    
for html_file in html_f_3:
    pdf_file = os.path.join(html_directory_main, html_file.replace('.html', '.pdf'))
    pdfkit.from_file(os.path.join(html_directory_main, html_file), pdf_file)

# Merge PDF files into a single document
# merged_pdf_file = html_directory_main
# pdf_files = [os.path.join(html_directory_main, f.replace('.html', '.pdf')) for f in html_files]
# pdfkit.from_file(pdf_files, merged_pdf_file)