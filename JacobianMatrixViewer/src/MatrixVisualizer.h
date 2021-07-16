#ifndef MatrixVisualizerH
#define MatrixVisualizerH

#include <limits>

#include <QPixmap>
#include <QPainter>
#include <QPrinter>

/*! Allows creating visualization of matrix structures and current values.
	This class is implemented using a policy-pattern and requires either
	* MatrixInterfaceSparse, or
	* MatrixInterfaceBand
	as template parameter as in the following example.
	\code
	// Template class MatrixInterfaceSparse must provide functions
	// setMatrix(mat), element(row, col), size()
	MatrixVisualizer<MatrixInterfaceSparse> matVis;
	matVis.setMatrix(mat);
	// export as bitmap
	matVis.gemeratePixmap().save("mat_pixmap.png");
	// print to QPrinter
	matVis.printVectorized(printer);
	\endcode
*/
template<typename MatrixInterface>
class MatrixVisualizer : public MatrixInterface {
public:
	/*! Default constructor, initializes class with meaningful defaults. */
	MatrixVisualizer() :
		m_cellPixelSize(2),
		m_cellPrintResolution(0),
		m_borderLinePixelWidth(1),
		m_borderLineWidth(0.25),
		m_zeroCellColor(QColor(194, 194, 255)),
		m_cellColor(Qt::black),
		m_backgroundColor(Qt::transparent)
	{
	}

	/*! Stores content of matrix into a pixmap.
		This function uses 3 colors:
		* m_backgroundColor (white/transparent) - cell is not being part of the matrix
		* m_zeroCellColor (light gray) - cell is part of the matrix structure, but value is zero
		* m_cellColor (black) - cell value is not zero
	*/
	QPixmap generatePixmap() const;

	/*! Prints the matrix to the printer. */
	void print(QPrinter & prn) const;

	/*! The number of pixels to be used for each cell/element of the matrix
		when exporting pixmaps.
	*/
	unsigned int	m_cellPixelSize;

	/*! Resolution in mm/cell to use for printing the matrix.
		The default is 0 (scale based on minimum of width/height of print area).
		Mind that using a too large value will draw outside of print margins.
	*/
	unsigned int	m_cellPrintResolution;

	/*! Border line width in mm for pixmap output.
		A border thickness of 0 disables border painting.
	*/
	unsigned int	m_borderLinePixelWidth;

	/*! Border line width in mm for vector output.
		A negative border line thickness disables border painting, a line
		thickness of 0 causes the painter to use a cosmetic pen.
	*/
	double			m_borderLineWidth;

	/*! Fill-color to be used for drawing cells that are part of the matrix pattern but are empty. */
	QColor			m_zeroCellColor;
	/*! Fill-color to be used for drawing cells with non-zero values (also used for border). */
	QColor			m_cellColor;
	/*! Background color to be used for pixmaps. */
	QColor			m_backgroundColor;
};


template<typename MatrixInterface>
QPixmap MatrixVisualizer<MatrixInterface>::generatePixmap() const {
	// compute size in pixels
	unsigned int n = MatrixInterface::size();
	unsigned int pixSize = n*m_cellPixelSize;
	// create pixmap with desired sizes but add pixels for boundary
	QPixmap pixmap(pixSize + 2*m_borderLinePixelWidth, pixSize + 2*m_borderLinePixelWidth);

	// fill pixmap with transparent color
	pixmap.fill( m_backgroundColor );

	QPainter p(&pixmap);

	// draw pixels
	p.setPen( m_cellColor );
	for (unsigned int i=0; i<n; ++i) {
		for (unsigned int j=0; j<n; ++j) {
			double val = MatrixInterface::value(i,j);
			if (val == std::numeric_limits<double>::infinity()) continue;
			if (val == 0) ///  \todo use "near zero" limit as customization parameter
				p.fillRect(m_borderLinePixelWidth + j*m_cellPixelSize, m_borderLinePixelWidth + i*m_cellPixelSize,
						   m_cellPixelSize, m_cellPixelSize, m_zeroCellColor);
			else
				p.fillRect(m_borderLinePixelWidth + j*m_cellPixelSize, m_borderLinePixelWidth + i*m_cellPixelSize,
						   m_cellPixelSize, m_cellPixelSize, m_cellColor);
		}
	}

	// draw boundary
	if (m_borderLinePixelWidth != 0) {
		QPen borderPen(m_cellColor);
		borderPen.setWidth(m_borderLinePixelWidth);

		p.setPen( borderPen );
		p.drawRect(m_borderLinePixelWidth/2,m_borderLinePixelWidth/2,
				   pixSize+1.5*m_borderLinePixelWidth, pixSize+1.5*m_borderLinePixelWidth);
	}

	return pixmap;
}


template<typename MatrixInterface>
void MatrixVisualizer<MatrixInterface>::print(QPrinter & prn) const {
	QPainter p(&prn);

	// get resolution in dpi
	int res = prn.resolution();
	// get number of dots
	int prnW = std::min(prn.width(), prn.height());
	// get number of cells
	unsigned int n = MatrixInterface::size();
	// compute dots per cell when using whole width/height
	double dotsPerCell = prnW/n;

	// if user has specified cell resolution, we need to compute the exact size
	if (m_cellPrintResolution > 0) {
		// get requested cell resolution in dpi, m_cellPrintResolution is in mm/cell
		dotsPerCell = res/25.4*m_cellPrintResolution;
		// dots/cell = dots/inch / ( 25.4 mm/inch ) * mm/cell
	}

	QPen pen(m_cellColor);
	// set cosmetic pen for drawing cells
	pen.setWidth(0);
	p.setPen( pen );

	// draw cells
	for (unsigned int i=0; i<n; ++i) {
		for (unsigned int j=0; j<n; ++j) {
			double val = MatrixInterface::value(i,j);
			if (val == std::numeric_limits<double>::infinity()) continue;
			if (val == 0) ///  \todo use "near zero" limit as customization parameter
				p.fillRect(dotsPerCell*j, dotsPerCell*i, dotsPerCell, dotsPerCell, m_zeroCellColor);
			else
				p.fillRect(dotsPerCell*j, dotsPerCell*i, dotsPerCell, dotsPerCell, m_cellColor);
			//p.drawRect(dotsPerCell*j, dotsPerCell*i, dotsPerCell, dotsPerCell);
		}
	}

	// draw border
	if (m_borderLineWidth >= 0) {
		double penWidth = res/25.4*m_borderLineWidth; // dots = dots/inch / (mm/inch) * 0.25 mm thickness
		pen.setWidthF(penWidth);
		p.setPen( pen );
		int w2 = dotsPerCell*n;
		p.drawRect(0,0, w2, w2);
	}
}



//void SparseMatrixEID::printPostScript(const std::string & filename,
//								   const std::string &title,
//								   double size,
//								   const unsigned int ptitle,
//								   const bool & unit,
//								   const bool & showValue,
//								   const bool & isStarTrekMode,
//								   const bool & highLightShortValues
//								   ) const {

//	std::ofstream out( filename.c_str() );

//	unsigned int nr,nc,maxdim,ltit;
//	double lrmrgn,botmrgn,xtit,ytit,ytitof,fnstit;
//	double xl,xr, yb,yt, scfct,u2dot,frlw,delt,paperx,xx,yy;

//	double conv = 2.54;
//	double haf = 0.5;
//	double zero = 0.0;
//	bool square = false;


//	// for now we have always ncol == nrow
//	// number of rows in matrix
//	unsigned int nrow = m_n;

//	// number of columns in matrix
//	unsigned int ncol = m_n;

//	// number of separation lines to draw for showing a partionning
//	// of the matrix. enter zero if no partition lines are wanted.
//	unsigned int nlines = 0;

//	// integer array of length nlines containing the coordinates of
//	// the desired partition lines . The partitioning is symmetric:
//	// a horizontal line across the matrix will be drawn in
//	// between rows lines(i) and lines(i)+1 for i=1, 2, ..., nlines
//	// an a vertical line will be similarly drawn between columns
//	// lines(i) and lines(i)+1 for i=1,2,...,nlines
//	std::vector<unsigned int> lines;
//	lines.resize(nlines);

//	nr =  nrow;
//	nc = ncol;

//	maxdim = std::max(nrow, ncol);
//	unsigned int m = 1 + maxdim;
//	nc = nc+1;
//	nr = nr+1;

//	// units (cm or in) to dot conversion factor and paper size
//	if ( unit ) {
//		u2dot = 72.0/conv;
//		paperx = 21.0;
//	} else {
//		u2dot = 72.0;
//		paperx = 8.5*conv;
//		size = size*conv;
//	}
//	// left and right margins (drawing is centered)
//	lrmrgn = (paperx-size)/2.0;

//	// bottom margin : 2 cm
//	botmrgn = 2.0;

//	// scaling factor
//	scfct = size*u2dot/m;

//	// matrix frame line witdh
//	frlw = 0.25;

//	// font size for title (cm)
//	fnstit = 0.5;
//	ltit = title.size();

//	// position of title : centered horizontally
//	// at 1.0 cm vertically over the drawing
//	ytitof = 1.0;
//	xtit = paperx/2.0;
//	ytit = botmrgn+size*nr/m + ytitof;

//	// almost exact bounding box
//	xl = lrmrgn*u2dot - scfct*frlw/2;
//	xr = (lrmrgn+size)*u2dot + scfct*frlw/2;
//	yb = botmrgn*u2dot - scfct*frlw/2;
//	yt = (botmrgn+size*nr/m)*u2dot + scfct*frlw/2;
//	if (ltit > 0) {
//		yt = yt + (ytitof+fnstit*0.70)*u2dot;
//	}

//	// add some room to bounding box
//	delt = 10.0;
//	xl = xl-delt;
//	xr = xr+delt;
//	yb = yb-delt;
//	yt = yt+delt;

//	// correction for title under the drawing
//	if (ptitle == 0 && ltit > 0) {
//		ytit = botmrgn + fnstit*0.3;
//		botmrgn = botmrgn + ytitof + fnstit*0.7;
//	}

//	// begin of output
//	out << "%!" << std::endl;
//	out << "%%Creator: printPostScript routine" << std::endl;
//	out << "%%BoundingBox: " << xl << " " << yb << " " << xr << " " << yt << std::endl;
//	out << "%%EndComments" << std::endl;
//	out << "/cm {72 mul 2.54 div} def" << std::endl;
//	out << "/mc {72 div 2.54 mul} def" << std::endl;
//	out << "/pnum { 72 div 2.54 mul 20 string cvs print ( ) print} def" << std::endl;
//	// calculate width of the string and move string to the left by half of its length
//	out << "/Cshow {dup stringwidth pop -2 div 0 rmoveto show} def" << std::endl;

//	// we leave margins etc. in cm so it is easy to modify them if
//	// needed by editing the output file
//	out << "gsave" << std::endl;
//	if ( ltit > 0 ) {
//		out << "/Helvetica findfont "<< fnstit << " cm scalefont setfont " << std::endl;
//		out << xtit << " cm " << ytit << " cm moveto " << std::endl;
//		out << "(" << title << ") Cshow" << std::endl;
//	}
//	out << "grestore" << std::endl;
//	out << lrmrgn << " cm " << botmrgn << " cm translate" << std::endl;
//	out << size << " cm " << m << " div dup scale " << std::endl;

//	// draw a frame around the matrix
//	out << frlw << " setlinewidth" << std::endl;
//	out << "newpath" << std::endl;
//	out << 0 << " " << 0 << " moveto" << std::endl;
//	if (square) {
//		out << m << " " << 0 << " lineto" << std::endl;
//		out << m << " " << m << " lineto" << std::endl;
//		out << 0 << " " << m << " lineto" << std::endl;
//	} else {
//		out << nc << " " << 0 << " lineto" << std::endl;
//		out << nc << " " << nr << " lineto" << std::endl;
//		out << 0 << " " << nr << " lineto" << std::endl;
//	}
//	out << "closepath stroke" << std::endl;

//	// drawing the separation lines

//	out << " 0.2 setlinewidth" << std::endl;

//	for ( unsigned int kol=0; kol < nlines; ++kol ) {

//		unsigned int isep = lines[kol];

//		// horizontal lines
//		yy =  double(nrow-isep) + haf;
//		xx = double(ncol+1);
//		out << zero << " " << yy << " moveto " << std::endl;
//		out << xx << " " << yy << " lineto stroke " << std::endl;

//		// vertical lines

//		xx = double(isep) + haf;
//		yy = double(nrow+1);
//		out << xx << " " << zero << " moveto " << std::endl;
//		out << xx << " " << yy << " lineto stroke " << std::endl;
//	}

//	 // plotting loop

//	out << "1 1 translate" << std::endl;
//	out << "0.8 setlinewidth" << std::endl;
//	out << "/p { 0 0 0 setrgbcolor moveto 0 -.40 rmoveto " << std::endl;
//	out << "           0  .80 rlineto stroke} def" << std::endl;
//	out << "/pg { 1 0 0 setrgbcolor moveto 0 -.40 rmoveto " << std::endl;
//	out << "            0  .80 rlineto stroke} def" << std::endl;

//	// we need  to replace 0.5 by a value we calculated by a string width (digits count)
//	out << "/t {moveto -.40 0 rmoveto} def" << std::endl;
//	out << "/Helvetica findfont 1 selectfont " << std::endl;

//	for (unsigned int i=0; i<m_n; ++i) {
//		for (unsigned int j=0; j<m_elementsPerRow; ++j) {
//			int index = m_index[(i*m_elementsPerRow)+j];
//			if ( !INVALID_INDEX(j, m_index, i*m_elementsPerRow)) {

//				if (!highLightShortValues){
//					out << std::fixed;
//				}

//				double value = m_data[(i*m_elementsPerRow)+j];
//				if ( !showValue ) {

//					if ( value != 0) {
//						out << index << " " << nrow-i-1 << " p" << std::endl;
//					} else {
//						out << index << " " << nrow-i-1 << " pg" << std::endl;
//					}

//				} else {

//					if ( value != 0) {

//						// todo find out more about relative fond scaling
//						out << "gsave" << std::endl;
//						out << index << " " << nrow-i-1 << " t" << std::endl;

//						if (highLightShortValues){
//							out << " (" << m_data[(i*m_elementsPerRow)+j] << ")" << std::endl;
//						} else {
//							out << " (" << std::setprecision(9) << m_data[(i*m_elementsPerRow)+j] << ")" << std::endl;
//						}

//						if (isStarTrekMode){
//							out << "dup stringwidth pop 0.5 add 1 exch div dup 2 mul scale"  << std::endl;
//						} else {
//							out << "dup stringwidth pop 0.5 add 1 exch div dup scale" << std::endl;
//						}
//						out << "show"  << std::endl;
//						out << "grestore" << std::endl;

//					} else {

//						/// \todo what we do with zeros? optional highlighting??

//					} // else if ( value != 0) {

//				} // else if ( showValue ) {

//			} // if ( index != -1 ){

//		} // for (unsigned int j=0; j<m_elementsPerRow; ++j) {
//	} // for (unsigned int i=0; i<m_n; ++i) {

//	out << "showpage" << std::endl;

//	out.close();

//}


#endif // MatrixVisualizerH
