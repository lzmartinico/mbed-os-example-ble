all: report

report: report.tex bibliography.bib
	latexmk -pdf report

slides: slides.tex
	latexmk slides

clean:
	latexmk -CA
