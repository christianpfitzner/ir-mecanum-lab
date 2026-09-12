# Bau der Praktikumsanleitung. latexmk sorgt dafür, dass Querverweise sitzen.
LATEXMK  := latexmk
SRC      := anleitung.tex figures/mecanum.tikz figures/architektur.tikz
KSRC     := kalman.tex

anleitung.pdf: $(SRC)
	$(LATEXMK) -pdf -interaction=nonstopmode anleitung.tex

kalman.pdf: $(KSRC)
	$(LATEXMK) -pdf -interaction=nonstopmode kalman.tex

.PHONY: build kalman clean watch log log-kalman

build: anleitung.pdf

kalman: kalman.pdf

watch:
	$(LATEXMK) -pdf -pvc anleitung.tex

log:
	@grep -nE "^!|Overfull \\\\hbox \(([1-9][0-9]|[0-9]+[0-9])pt|Undefined control|LaTeX Warning: (Reference|Citation)" anleitung.log || echo "log sauber"

log-kalman:
	@grep -nE "^!|Overfull \\\\hbox \(([1-9][0-9]|[0-9]+[0-9])pt|Undefined control|LaTeX Warning: (Reference|Citation)" kalman.log || echo "log sauber"

clean:
	-$(LATEXMK) -C >/dev/null 2>&1
	-$(LATEXMK) -C kalman.tex >/dev/null 2>&1
	rm -f *.aux *.log *.out *.toc *.lol *.fls *.fdb_latexmk *.synctex.gz \
	      *.lof *.bbl *.blg
