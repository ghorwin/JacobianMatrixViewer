# JacobianMatrixViewer

Displays Jacobian matrix structure, exports images and allows comparison between Jacobian matrix patterns.

This is a small tool to assist in developing numerical solver engines, where Newton-Raphson methods
are used with custom Jacobians. Hereby, using an incomplete Jacobian pattern, or incorrectly computed
pattern will easily hamper efficiency of the Newton method. Sometimes this leads to increased number of
convergence failures, but often enough the effects are subtle, and it is hard to identify these as mistakes.
Especially, since a well-written Newton-equation solver within an adaptive time step integrator will
still yield correct results - yet potentially much slower.

## Download and Installation

None, so far. Just download the source, fire up Qt Creator and build it yourself. I may add pre-compiled binaries once the tool is done.

## Usage

Ok, enough boilerplate - what can you do with the tool?

Upon start, you select a matrix file to analyse. This can be a dense reference matrix (generated with accurate, yet slow finite-difference quotient algorithm),
or a sparse matrix. The tool supports plain ASCII table-like data files and custom binary formats
matching the CSR-data format used in the IBK and IBKMK libraries (see IBKMK::SparseMatrixCSR).

Then, you can see the structure of the matrix pattern and can compare values.

Use the radio-buttons to switch between different views.

For example, look at the dense reference matrix:

<img src="doc/DenseReference.png" width="600px"/>

Green are cells with numbers different from zero and hence definitely part of the Jacobian pattern.
White are cells with zero, which can either mean "not part of the Jacobian pattern", or computed to be zero. 
With many physical codes zeros can appear in the Jacobian matrix when clipping takes place, so make sure 
that is avoided when dumping the Jacobian data.

Now take a look at a sparse matrix:

<img src="doc/SparseMatrixCSR.png" width="600px"/>

Here, you see cells with dark-gray background. These are not part of the Jacobian pattern.

Finally, you can compare the two (radio-button "Difference"):

<img src="doc/Difference.png" width="600px"/>

This is a bit more colorful:

- _orange_ : cells that are zero in one matrix and non-existend in the other; usually this means that these cells are not part of the Jacobian pattern
- _light blue_ : cells that have values in both matrixes and whose values differ only marginally (less than 1e-5)
- _blue_ : cells that have values in both matrixes and show significant differences -> this usually indicates a *bug* in the Jacobian calculation/composition algorithm
- _red_ : cells that have values in one matrix, yet are not part of the Jacobian pattern in the other matrix. This clearly indicates that the sparse matrix missing cells in its pattern.


## Binary matrix formats

TODO ...




