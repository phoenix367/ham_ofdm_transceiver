# Makefile for building the technical report

# Main document
MAIN = report
TEX = $(MAIN).tex
BIB = references.bib

# Build directory
BUILD_DIR = build
PDF = $(BUILD_DIR)/$(MAIN).pdf
PDF_OUT = $(BUILD_DIR)/OFDM_Transceiver_Technical_Report.pdf

# LaTeX compiler
LATEX = pdflatex
BIBTEX = bibtex
LATEX_FLAGS = -output-directory=$(BUILD_DIR) -interaction=nonstopmode

# Mermaid diagrams (architecture, chains, protocol sequences)
MERMAID_SRCS = $(wildcard images/*.mmd)
MERMAID_IMGS = $(MERMAID_SRCS:.mmd=.png)
PUPPETEER_CFG = images/puppeteer-config.json

# Measured plots copied from ../results (regenerate with the experiments)
RESULTS_DIR = ../results
RESULT_NAMES = ber_vs_snr.png per_vs_snr.png per_stf_vs_newman.png \
               per_adaptive_modes.png link_adaptation.png \
               system_demo_timeline.png system_demo_fixed_qam16_timeline.png \
               coarse_search_ab.png \
               zc_acf.png tx_signal.png \
               constellation_qpsk.png constellation_qam16.png \
               constellation_ladder.png llr_shape.png viterbi_recal.png ldpc_recal.png fec_comparison.png \
               extreme_recal.png qam16_recal.png \
               sdr_tx_spectrum.png sdr_drive_sweep.png \
               sdr_waterfall.png stream_mode.png broadcast_demo.png
RESULT_IMGS = $(addprefix images/,$(RESULT_NAMES))

ALL_DIAGRAMS = $(MERMAID_IMGS) $(RESULT_IMGS)

# Build the PDF (default target — must come before pattern rules)
all: $(ALL_DIAGRAMS) $(PDF_OUT)

images/%.png: images/%.mmd $(PUPPETEER_CFG)
	mmdc -i $< -o $@ -s 3 -b white --puppeteerConfigFile $(PUPPETEER_CFG)

$(RESULT_IMGS): images/%.png: $(RESULTS_DIR)/%.png
	cp $< $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(PDF_OUT): $(TEX) $(BIB) $(ALL_DIAGRAMS) $(wildcard sections/*.tex) $(wildcard appendices/*.tex) | $(BUILD_DIR)
	$(LATEX) $(LATEX_FLAGS) $(MAIN)
	cp $(BIB) $(BUILD_DIR)/
	-cd $(BUILD_DIR) && $(BIBTEX) $(MAIN)
	$(LATEX) $(LATEX_FLAGS) $(MAIN)
	$(LATEX) $(LATEX_FLAGS) $(MAIN)
	mv $(PDF) $(PDF_OUT)

# Quick build (without bibliography)
quick: | $(BUILD_DIR)
	$(LATEX) $(LATEX_FLAGS) $(MAIN)
	mv $(PDF) $(PDF_OUT)

# Clean auxiliary files
clean:
	rm -f $(BUILD_DIR)/*.aux $(BUILD_DIR)/*.log $(BUILD_DIR)/*.out $(BUILD_DIR)/*.toc $(BUILD_DIR)/*.bbl $(BUILD_DIR)/*.blg $(BUILD_DIR)/*.lot $(BUILD_DIR)/*.lof

# Clean everything including PDF and rendered diagrams
cleanall:
	rm -rf $(BUILD_DIR)
	rm -f images/*.png

# View the PDF
view: $(PDF_OUT)
	xdg-open $(PDF_OUT) 2>/dev/null || open $(PDF_OUT) 2>/dev/null || echo "Please open $(PDF_OUT) manually"

.PHONY: all quick clean cleanall view
