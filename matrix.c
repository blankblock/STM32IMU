/** @file matrix.c
 *		@brief 행렬 연산 관련 함수 정의
 *  	@date Created on: 2011. 1. 28.
 *      @author Original code created by CMU\n
 *      		Modified for Coretex M3 by Kyuhyong You
 */
#include <stdlib.h>
#include "matrix.h"

//void initMatrix(matrix *matrix, uint16_t Rows, uint16_t Cols)
//{
//	uint16_t i = 0;
//	uint16_t j = 0;
//	matrix->cols = Cols;
//	matrix->rows = Rows;
//	for(i=0; i<Rows; i++)
//	{
//		for(j=0; j<Cols; j++)
//		{
//			matrix->buf[i][j] = 0.0;
//		}
//	}
//}
/** @fn matrix newmat(uint16_t rows, uint16_t cols)
 * 		@brief 새로운 행렬 생성
 * 		@param rows 행 숫자 (가로 축)
 * 		@param cols 열 숫자 (세로 축)
 */
matrix newmat(uint16_t rows, uint16_t cols)
{
	int i;
	matrix mat;

	mat = (struct matrix_s *) malloc(sizeof (struct matrix_s));
	mat->rows = rows;
	mat->cols = cols;
	mat->buf = (float **) malloc(rows*sizeof(float *) );
	for(i=0 ; i<rows ; i++)
	{
		mat->buf[i] = (float *) malloc(cols*sizeof(float) );
	}
	return(mat);
}
/******************************************************************************
*******************************    FREEMAT         ****************************
*******************************************************************************

        Destroy a matrix.

        */
void freemat(matrix mat)
{
	int i;
	if( mat != NULL )
	{
		if(mat->buf != NULL)
		{
			for(i=0 ; i<mat->rows ; i++)
				free((char *)mat->buf[i]);
			free((char *)mat->buf);
		}
		free((char *)mat);
	}
}

/******************************************************************************
************************************  MATCOPY      ****************************
*******************************************************************************

        Copy a matrix. Semantics are A <= B.

        */
/** @fn void matcopy(matrix A, matrix B)
 * 		@brief B행열을 복사하여 A에 넣음
 * 		@param A 복사되는 행열
 * 		@param B 복사할 행열
 */
void matcopy(matrix A, matrix B)
{
	int i,j;

	for(i=0 ; i<A->rows; i++)
		for(j=0 ; j<A->cols; j++)
			m_el(A,i,j) = m_el(B,i,j);
}

/******************************************************************************
************************************  MATADD       ****************************
*******************************************************************************

        Add matrices. Semantics are A <= B + C.

        */

void matadd(matrix A, matrix B, matrix C)
{
	int i,j;

	matrix TB=NULL; /* absolutely must initialize to null */
	matrix TC=NULL; /* absolutely must initialize to null */

	if( A == B )
	{
		TB = newmat(B->rows,B->cols);
		matcopy(TB,B); B = TB;
	}
	if( A == C )
	{
		TC = newmat(C->rows,C->cols);
		matcopy(TC,C); C = TC;
	}

	//materr(A->rows!=B->rows,"matadd: error - AB size mismatch");
	//materr(A->cols!=B->cols,"matadd: error - AB size mismatch");
	//materr(B->rows!=C->rows,"matadd: error - BC size mismatch");
	//materr(B->cols!=C->cols,"matadd: error - BC size mismatch");
	for(i=0 ; i<A->rows; i++)
		for(j=0 ; j<A->cols ; j++)
			m_el(A,i,j) = m_el(B,i,j)+m_el(C,i,j);

	freemat(TB);
	freemat(TC);
}

/******************************************************************************
************************************  MATSUB       ****************************
*******************************************************************************

        Subtract matrices. Semantics are A <= B - C.

        */

void matsub(matrix A, matrix B, matrix C)
{
	int i,j;

	matrix TB=NULL; /* absolutely must initialize to null */
	matrix TC=NULL; /* absolutely must initialize to null */

	if( A == B )
	{
		TB = newmat(B->rows,B->cols);
		matcopy(TB,B); B = TB;
	}
	if( A == C )
	{
		TC = newmat(C->rows,C->cols);
		matcopy(TC,C); C = TC;
	}

	//materr(A->rows!=B->rows,"matsub: error - AB size mismatch");
	//materr(A->cols!=B->cols,"matsub: error - AB size mismatch");
	//materr(B->rows!=C->rows,"matsub: error - BC size mismatch");
	//materr(B->cols!=C->cols,"matsub: error - BC size mismatch");
	for(i=0 ; i<A->rows; i++)
	for(j=0 ; j<A->cols ; j++)
		m_el(A,i,j) = m_el(B,i,j)-m_el(C,i,j);

	freemat(TB);
	freemat(TC);
}

/******************************************************************************
************************************  MATMUL       ****************************
*******************************************************************************

        Multiply matrices. Semantics are A <= B * C.

        */

void matmul(matrix A, matrix B, matrix C)
{
	int i,j,k;
	double sum;

	matrix TB=NULL; /* absolutely must initialize to null */
	matrix TC=NULL; /* absolutely must initialize to null */

	//materr(B->cols!=C->rows,"matmul: error - BC size mismatch");
	//materr(A->rows!=B->rows,"matmul: error - AB size mismatch");
	//materr(A->cols!=C->cols,"matmul: error - AC size mismatch");

	if( A == B )
	{
		TB = newmat(B->rows,B->cols);
		matcopy(TB,B); B = TB;
	}
	if( A == C )
	{
		TC = newmat(C->rows,C->cols);
		matcopy(TC,C); C = TC;
	}

	for(i=0 ; i< B->rows; i++)
		for(j=0 ; j< C->cols ; j++)
		{
			sum = 0.0;
			for(k=0 ; k< B->cols ; k++)
				sum +=  m_el(B,i,k) * m_el(C,k,j);
			m_el(A,i,j) = sum;
		}

	freemat(TB);
	freemat(TC);
}
