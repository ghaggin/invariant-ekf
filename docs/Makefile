SOURCES =  report.tex proposal.tex
TARGET = $(basename $(SOURCES))
LATEX_FLAGS = -interaction=nostopmode -file-line-error
LATEX_ERROR_CODE = ".*:[0-9]*:.*"
all : $(TARGET)

% : %.tex
	@printf "    $@"
	@pdflatex $(LATEX_FLAGS) $< || true #| grep $(LATEX_ERROR_CODE) && exit 1 || true
	@bibtex $@ || true
	@pdflatex $(LATEX_FLAGS) $< || true #| grep $(LATEX_ERROR_CODE) && exit 1 || true
	@pdflatex $(LATEX_FLAGS) $< && echo ok #> /dev/null && echo ok
	@rm -rf $@.aux $@.log $@.out $@.bbl $@.blg
	
clean :
	@rm -f *.aux *.log *.pdf *.bbl *.synctex.gz *.fdb_latexmk *.fls $(PWD)/bin/*
	@echo "    ok"

.PHONY: all clean