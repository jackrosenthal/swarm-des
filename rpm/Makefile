DOCNAME=report
LATEX=xelatex
TEXFLAGS=

.PHONY: all
all: $(DOCNAME).pdf count

count: $(DOCNAME).pdf $(DOCNAME).tex
	@echo "*******************"
	@pdfinfo $(DOCNAME).pdf | grep "Pages"
	@texcount $(DOCNAME).tex | grep "in text"
	@echo "*******************"

$(DOCNAME).bmk:
	@touch $(DOCNAME).bmk

$(DOCNAME).bcf:
	@echo Preliminary run of LaTeX to generate biber files...
	@$(LATEX) --no-pdf -halt-on-error $(DOCNAME).tex >/dev/null || (cat $(DOCNAME).log && exit 1)
	@touch $(DOCNAME).bmk

$(DOCNAME).bbl: $(DOCNAME).bib $(DOCNAME).bcf $(DOCNAME).bmk
	@echo "Running Biber..."
	@biber $(DOCNAME) | (grep "^WARN" || exit 0)

$(DOCNAME).pdf: $(DOCNAME).tex
	@make $(DOCNAME).bbl
	@echo "Running LaTeX..."
	@($(LATEX) $(TEXFLAGS) -halt-on-error $(DOCNAME).tex >/dev/null && (grep -A3 "Warning" $(DOCNAME).log || exit 0)) || (cat $(DOCNAME).log && exit 1)
	@grep "Please (re)run Biber" $(DOCNAME).log >/dev/null && touch $(DOCNAME).bmk || exit 0
	@make $(DOCNAME).bbl
	@while grep -e 'Rerun to get ' -e 'Please rerun LaTeX' $(DOCNAME).log ; do echo "Rerunning LaTeX..." && $(LATEX) -halt-on-error $(TEXFLAGS) $(DOCNAME).tex >/dev/null; done

.PHONY: clean
clean:
	rm -f $(DOCNAME).log $(DOCNAME).nav $(DOCNAME).aux $(DOCNAME).out $(DOCNAME).snm $(DOCNAME).toc $(DOCNAME).vrb $(DOCNAME).bbl $(DOCNAME).bcf $(DOCNAME).blg $(DOCNAME).run.xml $(DOCNAME).bmk $(DOCNAME).xdv $(DOCNAME).pdf
