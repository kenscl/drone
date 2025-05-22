;; -*- lexical-binding: t; -*-

(TeX-add-style-hook
 "drone"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-class-options
                     '(("article" "10pt" "a4paper")))
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("fullpage" "") ("amsmath" "") ("amsfonts" "") ("amssymb" "") ("enumitem" "shortlabels") ("booktabs" "") ("graphicx" "") ("xcolor" "") ("hyperref" "") ("ulem" "") ("tikz" "") ("inputenc" "utf8") ("float" "")))
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "href")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperimage")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperbaseurl")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "nolinkurl")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "url")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "path")
   (add-to-list 'LaTeX-verbatim-macros-with-delims-local "path")
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
    "sec:power"
    "motors"
    "sec:software"))
 :latex)

