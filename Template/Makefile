TARGET = sunypoly_sample
MAIN = ${TARGET}.tex
BIB = ${TARGET}.bib

DVI = ${TARGET}.dvi
PS = ${TARGET}.ps
PDF = ${TARGET}.pdf

TEXFILES = ${MAIN} vita.tex abstract.tex acknowledgments.tex appendix.tex
BIBFILES = ${BIB}

all:	${PDF}

clean:	clean_tex clean_code

clean_tex:
	rm -f ${PDF} ${PS} ${DVI} ${TARGET}.aux ${TARGET}.lof ${TARGET}.log ${TARGET}.lot ${TARGET}.toc ${TARGET}.bbl ${TARGET}.blg *~

# If you use this makefile to build code that affects your thesis,
# edit the "Makefile.code" file for this.
clean_code:
	make -f Makefile.code clean

code:
	make -f Makefile.code

${DVI}:	clean_tex code ${TEXFILES} ${BIBFILES}
	latex ${MAIN}
	bibtex ${TARGET}
	latex ${MAIN}
	latex ${MAIN}

${PDF}:	${DVI}
	dvipdf ${DVI}

# Just in case you want a PostScript file for some reason
${PS}:	${DVI}
	dvips ${DVI}

# Just in case you prefer Troff
troff: 
	clear ; echo "\n\n\n\n\nBurn in the fire of a thousand demon armpit hairs!\n\n\n\n\n"
