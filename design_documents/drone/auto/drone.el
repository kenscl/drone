;; -*- lexical-binding: t; -*-

(TeX-add-style-hook
 "drone"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-class-options
                     '(("article" "10pt" "a4paper")))
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("fullpage" "") ("amsmath" "") ("amsfonts" "") ("amssymb" "") ("enumitem" "shortlabels") ("booktabs" "") ("graphicx" "") ("xcolor" "") ("hyperref" "") ("ulem" "") ("tikz" "") ("inputenc" "utf8") ("float" "")))
   (TeX-run-style-hooks
    "latex2e"
    "article"
    "art10"
    "fullpage"
    "amsmath"
    "amsfonts"
    "amssymb"
    "enumitem"
    "booktabs"
    "graphicx"
    "xcolor"
    "hyperref"
    "ulem"
    "tikz"
    "inputenc"
    "float")
   (TeX-add-symbols
    '("norm" 1)
    '("Vh" 1)
    '("V" 1)
    '("M" 1)
    "nl"
    "RR")
   (LaTeX-add-labels
    "sec:hardware"
    "structure"
    "sec:software"))
 :latex)

