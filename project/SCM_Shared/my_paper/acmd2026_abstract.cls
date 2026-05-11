%----------------------------------------------------------------------------------------------------
%
% Class file for the abstract of the
% 8th Asian Conference on Multibody System Dynamics
%
% MKM 10082009
% Zie 07042011
%
%----------------------------------------------------------------------------------------------------

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{acmd2026_abstract}[2025/11/26 ACMD abstract class]
\ProcessOptions\relax

\DeclareOption{10pt}{\OptionNotUsed}
\DeclareOption{12pt}{\OptionNotUsed}
\DeclareOption{twocolumn}{\OptionNotUsed}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}

\ProcessOptions\relax

\LoadClass[11pt,a4paper]{article}

\RequirePackage{ucs}
\RequirePackage[utf8x]{inputenc}
\RequirePackage{mathptmx}
\RequirePackage[T1]{fontenc}

\RequirePackage{graphicx}
\RequirePackage{verbatim}
\RequirePackage{tabularx}
\RequirePackage{amsmath,amsfonts,amssymb,amsthm,amsbsy,amsmath}
\RequirePackage{latexsym}
\RequirePackage{bm}
\RequirePackage{a4}
\RequirePackage{fancyhdr}
\RequirePackage{engord}
\RequirePackage{epsfig,vmargin,booktabs}
\RequirePackage{color}


\setpapersize{A4}  % needs vmargin package
\setmarginsrb{25mm}{5mm}{25mm}{10mm}{15mm}{0mm}{0mm}{0mm}
\parindent=0pt


%\def\author{\@ifnextchar[{\@getshortauthor}{\@getauthor}}
%\def\@getauthor#1{\gdef\@author{#1}\gdef\@shortauthor{#1}}
%\def\@getshortauthor[#1]#2{\gdef\@shortauthor{#1}\gdef\@author{#2}}
%\newcommand{\address}[1]{\gdef\@address{#1}}
%\newcommand{\keywords}[1]{\gdef\@keywords{#1}}
%\renewcommand{\abstract}[1]{\gdef\@abstract{#1}}


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SECTIONS
\renewcommand\section{\@startsection {section}{1}{\z@}{-3.5ex plus -1ex minus-.2ex}{0.2ex}{\normalsize\bf}}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


\AtBeginDocument{%
  \pagestyle{empty}
  \thispagestyle{fancy}
  \fancyhf{} % clear all header and footer fields
  \renewcommand{\headrulewidth}{0pt}
  \rhead{
    \scriptsize{
      The {\engordnumber{12}} Asian Conference on Multibody Dynamics
    }\\
    October 25-29, 2026, Okinawa, Japan \vspace{5mm}
}
}

\endinput
