all: report

report: report.tex bibliography.bib
	latexmk -pdf report

slides: slides.tex
	latexmk -pdf slides

clean:
	latexmk -CA
