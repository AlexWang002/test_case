import sys
import xml.etree.ElementTree
import html
import argparse
import pandas as pd

class Message:
    def __init__(self, file, line, category, number, text):
        self.file = file if file is not None else ''
        self.line = line
        self.category = category
        self.number = number
        self.text = text
        self.supplementals = []

class FileSummary:
    def __init__(self, filename):
        self.msg_count = 0
        self.error_count = 0
        self.warning_count = 0
        self.info_count = 0
        self.note_count = 0
        self.supplemental_count = 0
        self.misra_count = 0
        self.autosar_count = 0
        self.filename = filename

def parse_msgs(filename):
    tree = xml.etree.ElementTree.parse(filename)
    root = tree.getroot()
    msgs = []
    last_primary_msg = None
    for child in root:
        msg = Message(
            child.find('file').text,
            child.find('line').text,
            child.find('type').text,
            child.find('code').text,
            child.find('desc').text
        )
        if msg.category == "supplemental":
            last_primary_msg.supplementals.append(msg)
        else:
            last_primary_msg = msg
            msgs.append(msg)
    return msgs

def summarize_files(msgs):
    file_summaries = dict()
    for msg in msgs:
        if msg.file not in file_summaries:
            file_summaries[msg.file] = FileSummary(msg.file)
        file_summary = file_summaries[msg.file]

        if msg.category == 'error':
            file_summary.error_count += 1
        elif msg.category == 'warning':
            file_summary.warning_count += 1
        elif msg.category == 'info':
            file_summary.info_count += 1
        elif msg.category == 'note':
            file_summary.note_count += 1
        elif msg.category == 'supplemental':
            file_summary.supplemental_count += 1

        if msg.category != 'supplemental':
            file_summary.msg_count += 1

        if 'MISRA' in msg.text:
            file_summary.misra_count += 1
        if 'AUTOSAR' in msg.text:
            file_summary.autosar_count += 1
    return file_summaries

def build_html_table(column_headers, data_source, row_generator):
    out = ""
    out += "<table>"
    out += "<tr>"
    for header in column_headers:
        out += "<th scope=\"col\">"
        out += header
        out += "</th>"
    out += "</tr>"
    for item in data_source:
        out += "<tr>"
        row = row_generator(item)
        for data in row:
            out += "<td>"
            out += str(data)
            out += "</td>"
        out += "</tr>"
    out += "</table>"
    return out

def format_benign_zero(x):
    return str(x) if x != 0 else "<span class=\"zero\">" + str(x) + "</span>"

def emit_html(msgs):
    out = ""
    out += "<!DOCTYPE html><html>"
    out += "<head>"
    out += "<meta charset=\"utf-8\"><title>Report</title>"
    out += "<style>"
    out += "body { font-family: sans-serif; margin: 1em; }"
    out += "table { border-collapse: collapse; }"
    out += "td { padding: 0.25em; border: 1px solid #AAAAAA; }"
    out += "th { padding: 0.5em; }"
    out += ".filename { font-family: monospace; font-weight: bold; }"
    out += ".zero { color: #AAAAAA; }"
    out += "</style>"
    out += "</head>"
    out += "<body>"
    out += "<div>"
    out += "<h1>Report</h1>"
    out += "<h2>Summary</h2>"
    file_summaries = summarize_files(msgs)
    summary_total = FileSummary('Total')
    for file in file_summaries.values():
        summary_total.msg_count += file.msg_count
        summary_total.error_count += file.error_count
        summary_total.warning_count += file.warning_count
        summary_total.info_count += file.info_count
        summary_total.note_count += file.note_count
        summary_total.misra_count += file.misra_count
        summary_total.autosar_count += file.autosar_count
    file_summaries['Total'] = summary_total
    out += build_html_table(
        ['File', 'Messages','Error','Warning','Info','Note','MISRA', 'AUTOSAR'],
        file_summaries.values(),
        lambda file: [
            ("<span class=\"filename\">" + html.escape(file.filename) + "</span>") if file.filename != 'Total' else file.filename,
            format_benign_zero(file.msg_count),
            format_benign_zero(file.error_count),
            format_benign_zero(file.warning_count),
            format_benign_zero(file.info_count),
            format_benign_zero(file.note_count),
            format_benign_zero(file.misra_count),
            format_benign_zero(file.autosar_count)
        ]
    )
    out += "<br>"
    out += "<h2>Details</h2>"
    out += build_html_table(
        ['File', 'Line', 'Category', '#', 'Description'],
        msgs,
        lambda msg: [
            "<span class=\"filename\">" + html.escape(msg.file) + "</span>",
            msg.line,
            msg.category,
            msg.number,
            html.escape(msg.text)
        ]
    )
    out += "</div>"
    out += "</body>"
    out += "</html>\n"
    return out

def format_benign_zero_excel(value):
    return str(value) if value != 0 else ""

def emit_excel(msgs, excel_file_name):
    file_summaries = summarize_files(msgs)
    summary_total = FileSummary('Total')
    for file in file_summaries.values():
        summary_total.msg_count += file.msg_count
        summary_total.error_count += file.error_count
        summary_total.warning_count += file.warning_count
        summary_total.info_count += file.info_count
        summary_total.note_count += file.note_count
        summary_total.misra_count += file.misra_count
    file_summaries['Total'] = summary_total

    summary_data = []
    for file in file_summaries.values():
        summary_data.append([
            file.filename,
            format_benign_zero_excel(file.msg_count),
            format_benign_zero_excel(file.error_count),
            format_benign_zero_excel(file.warning_count),
            format_benign_zero_excel(file.info_count),
            format_benign_zero_excel(file.note_count),
            format_benign_zero_excel(file.misra_count),
            format_benign_zero_excel(file.autosar_count)
        ])
    summary_df = pd.DataFrame(summary_data, columns=[
        'File', 'Messages','Error','Warning','Info','Note','MISRA C++', 'AUTOSAR'
    ])

    details_data = []
    for msg in msgs:
        details_data.append([
            msg.file,
            msg.line,
            msg.category,
            msg.number,
            msg.text
        ])
    details_df = pd.DataFrame(details_data, columns=[
        'File', 'Line', 'Category', '#', 'Description'
    ])

    with pd.ExcelWriter(excel_file_name, engine='openpyxl') as writer:
        summary_df.to_excel(writer, sheet_name='Summary', index=False)
        details_df.to_excel(writer, sheet_name='Details', index=False)

def emit_text_msg(msg):
    out = ""
    if (msg.file and msg.file != "") or (msg.line and msg.line != '0'):
        out += msg.file + " " + str(msg.line) + " "
    out += msg.category + " " + str(msg.number) + ": "
    out += msg.text + "\n"
    for supplemental in msg.supplementals:
        out += emit_text_msg(supplemental)
    return out

def emit_text(msgs):
    out = ""
    for msg in msgs:
        out += emit_text_msg(msg)
    return out

def write_output(output, filename):
    with open(filename, 'w') as file:
        file.write(output)

def main():
    parser = argparse.ArgumentParser(description='Generate HTML or text output from PC-lint Plus XML reports')
    parser.add_argument('--input-xml', action='store', help='XML input filename', required=True)
    parser.add_argument('--output-text', action='store', help='Text output filename', required=False)
    parser.add_argument('--output-html', action='store', help='HTML output filename', required=False)
    parser.add_argument('--output-excel', action='store', help='Excel output filename', required=False)
    args = parser.parse_args()

    if not (args.output_text or args.output_html):
        parser.error("no output destination specified")

    msgs = parse_msgs(args.input_xml)
    msgs.sort(key=lambda msg: (msg.file == "", msg.file, int(msg.line) if msg.line != "" else 0))
    if args.output_text:
        write_output(emit_text(msgs), args.output_text)
    if args.output_html:
        write_output(emit_html(msgs), args.output_html)
    if args.output_excel:
        emit_excel(msgs, args.output_excel)

if __name__ == "__main__":
    main()
