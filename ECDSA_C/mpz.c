#define _CRT_SECURE_NO_WARNINGS
#include <stdlib.h>
#include <time.h>
#include "mpz.h"

/******************************/
/*                            */
/*       1. MPZ  MACROs       */
/*                            */
/******************************/

#define MAC_ULONG_MULT(/* UINT */a,/* UINT */b,/* UINT */c,/* UINT */d) \
{\
	UINT __x1,__x2,__x3,__x4;\
	__x1 = MAC_LW(a)*MAC_LW(b);\
	__x2 = MAC_LW(a)*MAC_HW(b);\
	__x3 = MAC_HW(a)*MAC_LW(b);\
	__x4 = MAC_HW(a)*MAC_HW(b);\
	__x2 += __x3;\
	if(__x2 < __x3) __x4 += HIGHER_LSB_ONE;\
	__x4 += MAC_HW(__x2);\
	__x3 = MAC_LW(__x2)<<HALF_LONG_BITS;\
	__x1 += __x3;\
	if(__x1 < __x3) __x4++;\
	__x1 += (d);\
	if( __x1 < (d)) __x4++;\
	(d) = __x4;\
	(c) = __x1;\
}

#define MAC_ULONG_MULT_ADD(/* UINT */a,/* UINT */b,/* UINT */c,/* UINT */d) \
{\
	UINT __x1,__x2,__x3,__x4;\
	__x1 = MAC_LW(a)*MAC_LW(b);\
	__x2 = MAC_LW(a)*MAC_HW(b);\
	__x3 = MAC_HW(a)*MAC_LW(b);\
	__x4 = MAC_HW(a)*MAC_HW(b);\
	__x2 += __x3;\
	if(__x2 < __x3) __x4 += HIGHER_LSB_ONE; \
	__x4 += MAC_HW(__x2);\
	__x3 = MAC_LW(__x2)<<HALF_LONG_BITS;\
	__x1 += __x3;\
	if(__x1 < __x3) __x4++;\
	__x1 += (d);\
	if( __x1 < (d)) __x4++;\
	__x1 += (c);\
	if( __x1 < (c)) __x4++;\
	(d) = __x4;\
	(c) = __x1;\
}

#define MAC_ULONG_SQUARE(/*(UINT)*/ a,/*(UINT)*/ b,/*(UINT)*/ c) \
{\
	UINT __h,__m,__l;\
	__l = MAC_LW(a);\
	__h = MAC_HW(a);\
	__m = __l*__h;\
	__h *= __h;\
	__l *= __l;\
	__h += __m >> (HALF_LONG_BITS - 1);\
	__m = __m << (HALF_LONG_BITS + 1);\
	__l += __m;\
	if( __l < __m ) __h++;\
	b = __l;\
	c = __h;\
}

/*
	Specialized sigle precision division preparing for multiple precision division
*/
#define MAC_ULONG_DIVIDE(/*(UINT)*/ a,/*(UINT)*/ b,/*UINT*/ c,/*(UINT)*/ d) \
{\
	UINT __x3x2,__x1x0,__y1,__y0,__q,__z2z1,__z1z0,__tmp;\
	SINT  __cnt=2;\
	__x3x2 = (b) ;__x1x0 = (a);\
	__y1=MAC_HW(c);__y0=MAC_LW(c);\
	if( __x3x2 > (c) ){	__x3x2 -=__y1; }\
	while(1){\
		__q = ((__x3x2 == __y1) ? BITMASK_LOWER_LONG : __x3x2/__y1);\
		while(1){\
			__tmp = __x3x2 - __q*__y1;\
			if( (__tmp & BITMASK_HIGHER_LONG) || ( (__q*__y0) <= ((__tmp<<HALF_LONG_BITS)+ MAC_HW(__x1x0))))\
				break;\
			__q--;\
		}\
		__z2z1 = __q*__y1; __z1z0 = __q*__y0;\
		__tmp= (__z1z0>>HALF_LONG_BITS);\
		__z1z0 = (__z1z0<<HALF_LONG_BITS)&BITMASK_HIGHER_LONG;\
		__z2z1 +=__tmp;\
		if( __x1x0 < __z1z0) __z2z1++; /* or hi-- */\
		__x1x0 -= __z1z0;\
		if(__x3x2 < __z2z1){ __z2z1 += c; __q --;}\
		__x3x2 -= __z2z1;\
		if(--__cnt == 0) break;\
		(d) = __q << HALF_LONG_BITS;\
		__x3x2 = ((__x3x2 << HALF_LONG_BITS) | (__x1x0 >> HALF_LONG_BITS)) & BITMASK_LONG;\
		__x1x0 = (__x1x0 & BITMASK_LOWER_LONG) << HALF_LONG_BITS;\
	}\
	(d) |= __q;\
}

void MAC_ULONG_DIVIDE1(UINT *a, UINT *b, UINT *c, UINT *d)
{
	UINT __x3x2, __x1x0, __y1, __y0, __q, __z2z1, __z1z0, __tmp;
	SINT  cnt = 2;
	__x3x2 = (*b); __x1x0 = (*a);
	__y1 = MAC_HW(*c); __y0 = MAC_LW(*c);
	if ((__x3x2 & 0xffffffff) > ((*c) & 0xffffffff)) { __x3x2 -= __y1; }
	while (1) {
		__q = ((__x3x2 == __y1) ? BITMASK_LOWER_LONG : ((__x3x2 & 0xffffffff) / (__y1 & 0xffffffff) & 0xffffffff));
		while (1) {
			__tmp = __x3x2 - __q*__y1;
			if (((__tmp & 0xffffffff) & BITMASK_HIGHER_LONG) || (((__q*__y0) & 0xffffffff) <= ((((__tmp & 0xffffffff) << HALF_LONG_BITS) + MAC_HW(__x1x0)) & 0xffffffff)))
				break;
			__q--;
		}
		__z2z1 = (__q)*__y1; __z1z0 = __q*__y0;
		__tmp = ((__z1z0 & 0xffffffff) >> HALF_LONG_BITS);
		__z1z0 = (((__z1z0 & 0xffffffff) << HALF_LONG_BITS) & 0xffffffff)&BITMASK_HIGHER_LONG;
		__z2z1 += __tmp;
		if (((__x1x0 & 0xffffffff)) < ((__z1z0 & 0xffffffff))) { __z2z1++; /* or hi-- */ }
		__x1x0 -= __z1z0;
		if ((__x3x2 & 0xffffffff) < (__z2z1 & 0xffffffff)) { __z2z1 += *c; __q--; }
		__x3x2 -= __z2z1;
		if (--cnt == 0) break;
		(*d) = (__q & 0xffffffff) << HALF_LONG_BITS;
		__x3x2 = (((__x3x2 & 0xffffffff) << HALF_LONG_BITS) | (((__x1x0 & 0xffffffff) >> HALF_LONG_BITS) & 0xffffffff)) & BITMASK_LONG;
		__x1x0 = ((__x1x0 & 0xffffffff) & BITMASK_LOWER_LONG) << HALF_LONG_BITS;
	}
	(*d) = ((*d) & 0xffffffff) | (__q & 0xffffffff);
}



/******************************/
/*                            */
/*      2. MPZ Functions      */
/*                            */
/******************************/

/******************************/
/*      2.1 Utility Functions */
/******************************/

SINT
MPZ_copy(MPZ *src,MPZ *dest)
{
	SINT i,src_len;
	UINT *dest_datp,*src_datp;

	if(MAC_IS_MPZ_ZERO(*src)){
		MAC_MAKE_ZERO(*dest);
		return 0;
	}
	src_len = src->len;
	dest_datp = dest->dat;
	src_datp = src->dat;
	for(i = 0 ; i < src_len;i++){
		(dest->dat)[i] = (src->dat)[i];
	}
	dest->sig = src->sig;
	dest->len = src->len;
	return i;
}

SINT 
MPZ_shl_1bit(MPZ *a, MPZ *b)
{
	UINT c, *a_datp, *b_datp, tmp;
	SINT i;

	c = 0;
	a_datp = a->dat;
	b_datp = b->dat;
	b->sig = a->sig;
	b->len = a->len;

	for (i = 0; i < a->len; i++)
	{
		tmp = *(a_datp++);
		*(b_datp++) = (tmp << 1) | c;
		c = (tmp & HIGHER_MSB_ONE) ? 1 : 0;
	}
	if (c)
	{
		*b_datp = 1;
		b->len++;
	}
	return 0;
}

SINT
MPZ_shl(MPZ *a, int n,MPZ *r)
{
	SINT i, nw, lb, rb;
	UINT *t, *f;
	UINT l;

	r->sig = a->sig;

#if 0
	nw = n / LONG_BITS;
	lb = n % LONG_BITS;
#else
	nw = n / LONG_BITS;
	lb = n - nw*LONG_BITS;
#endif

	rb = LONG_BITS - lb;
	f = a->dat;
	t = r->dat;
	t[a->len + nw] = 0;
	if(lb == 0){
		for(i = a->len - 1; i >= 0; i --)
			t[nw + i] = f[i];
	}else{
		for(i = a->len - 1; i >= 0; i --) 
		{
			l = f[i];
			t[nw+i+1] |= (l >> rb) & BITMASK_LONG;
			t[nw+i] = (l << lb) & BITMASK_LONG;
		}
	}
	memset(t, 0, nw * sizeof(t[0])); 

	r->len = a->len + nw + 1;
	
	MAC_CLR_UPPER_ZEROBYTES(*r);
	return 0;
}

SINT
MPZ_shr(MPZ *a, int n,MPZ *r)
{	
	SINT i;
	SINT  nw, nb;
	UINT *rd, *ad;
	UINT l, t;

#if 0
	nw = n / LONG_BITS;
	nb = n % LONG_BITS;
#else
	nw = n / LONG_BITS;
	nb = n - nw*LONG_BITS;
#endif

	if(nw > a->len){
		MAC_CLR_UPPER_ZEROBYTES(*r);
		return 0;
	}
	if(r != a)
		r->sig = a->sig;
	
	ad = &(a->dat[nw]);
	rd = r->dat;
	r->len = a->len - nw;
	
	if(nb == 0){
		for(i = a->len - nw + 1; (SINT)i > 0; i --)
			*(rd++)= *(ad++);
	}else{
		l = *(ad++);
		for(i = 1; i < a->len - nw; i ++){
			t = (l >> nb) & BITMASK_LONG;
			l= *(ad++);
			*(rd++) = t | ((l << (LONG_BITS - nb)) & BITMASK_LONG);
		}
		*(rd++) = (l >> nb) & BITMASK_LONG;
	}
	*rd = 0;
	MAC_CLR_UPPER_ZEROBYTES(*r);
	return 0;
}

SINT
MPZ_nonzero_bits_num(MPZ *a)
{
	UINT l;
	int i;

	if(a->len == 0) 
		return(0);
	l = a->dat[a->len-1];
	i = (a->len-1) * 32;

	if((l == 0 )&& (a->len-1 == 0) ){
		MAC_CLR_UPPER_ZEROBYTES(*a);
		l = a->dat[a->len-1];
		i = (a->len-1) * 32;
	}

	if(l & 0xffff0000L)
	{
		if(l & 0xff000000L)
			return(i + bits_to_index[l>>24L]+24);
		else	
			return(i + bits_to_index[l>>16L]+16);
	}else{
		if(l & 0xff00L)
			return(i + bits_to_index[l>>8]+8);
		else
			return(i + bits_to_index[l   ]  );
	}
}


/******************************/
/*   2.2 Interger comparison  */
/******************************/

SINT
MPZ_abs_comp(MPZ *a,MPZ *b)
{
	SINT i,cmplen;
	UINT *a_datp,*b_datp;

	if( cmplen = a->len - b->len)
		return ( (cmplen > 0) ? 1:-1);

	a_datp = a->dat;
	b_datp = b->dat;

	for(i=a->len-1;i >=0 ; i--){
		if(a_datp[i] != b_datp[i]){
			return ( ( a_datp[i] > b_datp[i] )? 1:-1);
		}
	}
	return 0;
}

SINT 
MPZ_comp(MPZ *a,MPZ *b)
{
	if( a->sig == b->sig ){
		if( a->sig == 0 )
			return 0; /* the case a=b=0 */
		else
			return (a->sig) * MPZ_abs_comp(a,b);
	}else{
		return a->sig - b->sig;
	}
}


/******************************/
/*   2.3 Interger additions   */
/******************************/

SINT 
MPZ_asym_add(MPZ *a,MPZ *b,MPZ *c)
{
	UINT carry,tmp;
	SINT i ,a_len,b_len;
	UINT *a_datp,*b_datp,*c_datp;
	
	
	a_len = a->len;
	b_len = b->len;
	a_datp = a->dat;
	b_datp = b->dat;
	c_datp = c->dat;

	carry = 0;
	for (i = 0; i < b_len; i++){
		tmp = a_datp[i];
		c_datp[i] = tmp + b_datp[i] + carry;

		carry = tmp  > c_datp[i] - carry ;
	}

	if(carry){
		for(;i< a_len;i++){
			tmp = a_datp[i];
			c_datp[i] = tmp + carry;
			if(tmp > c_datp[i]) carry = 1;
			else carry = 0;
		}
		if(carry){
			c_datp[i] = 1;
			i++;
		}
	}else{
		for(;i< a_len;i++)
			c_datp[i] = a_datp[i];
	}
	c->len = i;
	return 0;
}


/******************************/
/*  2.4  Interger subtractions */
/******************************/

SINT 
MPZ_asym_sub(MPZ *a,MPZ *b,MPZ *c)
{
	UINT borrow ;
	SINT i,a_len,b_len ;
	UINT *a_datp,*b_datp,*c_datp;
	borrow = 0;

	a_len = a->len;
	b_len = b->len;
	a_datp = a->dat;
	b_datp = b->dat;
	c_datp = c->dat;

	for( i = 0 ; i < b->len ; i ++ ){
		if(borrow){
			borrow = a_datp[i] <= b_datp[i] ;
			c_datp[i] = a_datp[i] - b_datp[i] - 1 ;
		}else{
			borrow = a_datp[i] < b_datp[i] ;
			c_datp[i] = a_datp[i] - b_datp[i] ;
		}
	}

	if(borrow)
	{
		for(; a_datp[i]==0 ; i++){
			c_datp[i] = a_datp[i] - 1;
			if( i >= a_len - 1) break;
		}
		c_datp[i] = a_datp[i] - 1;
		i++;
	}
	if(a->len > i )
	memcpy((char*)&(c_datp[i]),(char*)&(a_datp[i]),sizeof(UINT)*(a_len - i));
	c->len = a->len;

	return 0 ;
}

SINT 
MPZ_add(MPZ *a,MPZ *b,MPZ *c)
{
	SINT sigx;
	
	sigx = a->sig * b->sig;

	if( sigx >= 0 ){  
		if(a->len >= b->len ){
			MPZ_asym_add(a,b,c);
			c->sig = a->sig;
		}else{
			MPZ_asym_add(b,a,c);
			c->sig = b->sig;
		}
		
	}else{
		if( (sigx=MPZ_abs_comp(a,b)) >= 0){
			MPZ_asym_sub(a,b,c);
			c->sig = a->sig;
		}else if(sigx < 0 ){
			MPZ_asym_sub(b,a,c);
			c->sig = b->sig;
		}
	}
	
	return 0;
}

SINT
MPZ_sub(MPZ *a,MPZ *b,MPZ *c)
{
	SINT sigx;

	sigx = a->sig*b->sig;
	
	if(sigx <= 0 ){
		if(a->len >= b->len ){
			MPZ_asym_add(a,b,c);
		}else{
			MPZ_asym_add(b,a,c);
		}
		c->sig = (a->sig!=0)?a->sig:-b->sig;
	}else{
		if(MPZ_abs_comp(a,b)>=0){
			MPZ_asym_sub(a,b,c);
			c->sig = a->sig;
		}else{
			MPZ_asym_sub(b,a,c);
			c->sig = -b->sig;
		}
	}
	MAC_CLR_UPPER_ZEROBYTES(*c);
	return 0;
}


/*********************************/
/*  2.5 Interger multiplications */
/*********************************/

SINT
MPZ_mult_ULONG(MPZ *a, UINT b, MPZ *c)
{
	UINT higher;
	SINT i, a_len;
	UINT *a_datp, *c_datp;

	higher = 0;
	a_len = a->len;
	a_datp = a->dat;
	c_datp = c->dat;

	for (i = 0; i<a_len; i++) {
		MAC_ULONG_MULT(a_datp[i], b, c_datp[i], higher);
	}
	if (higher) {
		c_datp[i] = higher;
		c->len = i + 1;
	}
	else {
		c->len = i;
	}

	return 0;
}

SINT
MPZ_mult_ULONG_add(MPZ *a,UINT b,MPZ *c)
{
	UINT higher,tmp;
	SINT i,a_len,c_len;
	UINT *a_datp,*c_datp;

	higher=0;
	a_len = a->len;
	c_len = c->len;
	a_datp = a->dat;
	c_datp = c->dat;

	if( c_len <= a_len ){
		for(i=a_len;i >= c_len;i--){
			c_datp[i] = 0;
		}
	}/*Bug fixed: Check out of range of 'c'. */

	for(i=0;i<a_len;i++){
		MAC_ULONG_MULT_ADD(a_datp[i],b,c_datp[i],higher);
	}
	tmp = c_datp[i] + higher;
	higher = (tmp < c_datp[i]);

	if(higher){
		c_datp[i] = tmp;
		i++;
		while(1){
			tmp = c_datp[i] + 1;
			if (tmp > c_datp[i]){
				c_datp[i] = tmp;
				break;
			}
			c_datp[i] = tmp;
			i++;
		}
	}else{
		c_datp[i]=tmp;
	}
	c->len = MAC_MAX(i+1/* when c->len <= a->len */,c->len);

	return 0;
}

SINT
MPZ_plain_mul(MPZ *a, MPZ *b, MPZ *c)
{
	SINT i, b_len;
	MPZ tc;
	UINT *b_datp;

	b_len = b->len;
	b_datp = b->dat;

	memset(c->dat, 0, ((a->len + b->len) << 2) + 4);
	c->sig = a->sig * b->sig;
	c->len = a->len + b->len;
	tc.dat = c->dat;

	MPZ_mult_ULONG(a, b_datp[0], &tc);

	tc.dat++;
	tc.len--;
	for (i = 1; i < b_len; i++) {
		MPZ_mult_ULONG_add(a, b_datp[i], &tc);
		tc.dat++;
	}
	MAC_CLR_UPPER_ZEROBYTES(*c);

	return 0;
}

SINT
MPZ_plain_sqr(MPZ *a,MPZ *b)
{
	SINT i,j,k;
	SINT m,n;
	UINT higher;
	UINT carry,tmp,*a_datp,*b_datp;
	UINT tmp_dat[MAX_MPZ_BUF_LEN];

	m = a->len;
	n = (m<<1) - 1;
	carry = 0;
	a_datp = a->dat;
	b_datp = b->dat;
	memset((char*)tmp_dat,0,(a->len<<3)+4);

	for ( i = 0 ; i < m ; i++ ){
		MAC_ULONG_SQUARE(a_datp[i],b_datp[i<<1],b_datp[(i<<1)+1]);
	}
	/* k MUST be initialized as ZERO since k may not be evaluated in the loop when m = 1 */
	k = 0;
	for( i = 0 ; i < m ; i++ ){
		higher = 0;
		for( j = i+1 ; j < m ; j++){
			k=i+j;
			MAC_ULONG_MULT_ADD(a_datp[i],a_datp[j],tmp_dat[k],higher);
		}
		tmp_dat[k+1] +=higher;
	}
	
	/* ' * 2 ' can be replaced by shift-left one bit */
	n++;
	for ( i = n ; i > 0 ; i--){
		tmp_dat[i] = tmp_dat[i]<<1;
		if( tmp_dat[i-1] & HIGHER_MSB_ONE ){
			tmp_dat[i] |= LOWER_LSB_ONE;
		}
	}
	tmp_dat[0] = tmp_dat[0]<<1;

	for( i = 0 ; i <= n ; i++){
		tmp = tmp_dat[i];
		b_datp[i] += tmp + carry;
		carry = tmp  > b_datp[i] ;
	}
	
	b->sig = MPCONST_POS_SIG;
	b->len = n;

	MAC_CLR_UPPER_ZEROBYTES(*b);

	return 0;
}


/*********************************/
/*     2.6 Interger divisions    */
/*********************************/

SINT
MPZ_div(MPZ *a,MPZ *b,MPZ *c,MPZ *d)
{
	UINT normbits;
	MPZ dvder,dvdnd,window,tmp_dvdnd;
	UINT dvder_dat[MAX_MPZ_BUF_LEN],dvdnd_dat[MAX_MPZ_BUF_LEN],tmp_dvdnd_dat[MAX_MPZ_BUF_LEN];	
	UINT dvder1st,dvder2nd,tmp_quota,x3,x2,x1;
	SINT i;
	SINT cmp_result;
	SINT loop;

	dvder.dat=dvder_dat;
	dvdnd.dat=dvdnd_dat;
	tmp_dvdnd.dat=tmp_dvdnd_dat;

	/* For trivial cases */
	MAC_CLR_UPPER_ZEROBYTES(*a);MAC_CLR_UPPER_ZEROBYTES(*b);
	cmp_result=MPZ_abs_comp(b,a);
	if(cmp_result > 0 ){
		MPZ_copy(a,d);
		MAC_MAKE_ZERO(*c);
		return 0;
	}else if(cmp_result == 0 ){
		MAC_MAKE_ZERO(*d);
		c->dat[0]= 1;
		c->len = 1;
		return 0;
	}
	/* normalization of the dividend and the dvider */
	normbits=(LONG_BITS - MPZ_nonzero_bits_num(b))%LONG_BITS;
	MPZ_shl(b,normbits,&dvder);
	MPZ_shl(a,normbits,&dvdnd);
	dvder1st = dvder.dat[dvder.len-1];
	dvder2nd = (dvder.len == 1)? 0 : dvder.dat[dvder.len-2];
	window.dat = &(dvdnd.dat[dvdnd.len - dvder.len]);
	window.len = dvder.len;
	loop = c->len = dvdnd.len - dvder.len;
	if((cmp_result=MPZ_abs_comp(&window,&dvder))>=0){
		MPZ_asym_sub(&window,&dvder,&window);
		c->len++;
		c->dat[c->len - 1] = 1;
	}
	window.len=dvder.len+1;
	for(i = loop - 1  ; i >= 0 ; i--){
		window.dat--;
		window.len=dvder.len+1;
		if( window.dat[window.len-1] == dvder1st )
			tmp_quota = BITMASK_LONG;
		else
			MAC_ULONG_DIVIDE(window.dat[window.len-2],window.dat[window.len-1],dvder1st,tmp_quota);
		while(1){
			x3=0; x2=0; /* must be initialized every time */
			MAC_ULONG_MULT(dvder2nd,tmp_quota,x1,x3);
			MAC_ULONG_MULT_ADD(dvder1st,tmp_quota,x2,x3);
			if(window.dat[window.len-1] > x3) break;
			if(window.dat[window.len-1] == x3 ){
				 if( window.dat[window.len-2] > x2 ) break;
				 if( window.dat[window.len-2] == x2 ){
					if( window.dat[window.len-3] >= x1 ) break;
				 }
			}
			tmp_quota--;
		}
		MPZ_mult_ULONG(&dvder,tmp_quota,&tmp_dvdnd);
		if(MPZ_abs_comp(&window,&tmp_dvdnd)>=0){
			MPZ_asym_sub(&window,&tmp_dvdnd,&window);
			if(window.dat[window.len-1]){
				MPZ_asym_sub(&window,&dvder,&window);
				tmp_quota++;
			}
		}else{
			/* Do NOT change the order of operations */
			/* following operations are asymmetric   */
			MPZ_asym_add(&window,&dvder,&window);
			MPZ_asym_sub(&window,&tmp_dvdnd,&window);
			tmp_quota--;
		}
		c->dat[i]=tmp_quota;
	}
	if(d){
		if(normbits){
			MPZ_shr(&dvdnd,normbits,d);
		}else{
			MPZ_copy(&dvdnd,d);
		}
		if(MAC_IS_MPZ_ZERO(*d)){
			d->sig=MPCONST_ZERO_SIG;
		}else{
			d->sig=a->sig;
		}
	}

	c->sig=a->sig*b->sig;
	
	return 0;
}


/**********************************/
/*    2.7 MPZ modular arithmetic  */
/**********************************/

SINT
MPZ_mod(MPZ *a,MPZ *m,MPZ *r)
{
	UINT normbits;
	MPZ dvder,dvdnd,window,tmp_dvdnd;
	UINT dvder_dat[MAX_MPZ_BUF_LEN],dvdnd_dat[MAX_MPZ_BUF_LEN],tmp_dvdnd_dat[MAX_MPZ_BUF_LEN];
	UINT dvder1st,dvder2nd,tmp_quota,x3,x2,x1;
	SINT i;
	SINT cmp_result;
	SINT loop;

	dvder.dat=dvder_dat;
	dvdnd.dat=dvdnd_dat;
	tmp_dvdnd.dat=tmp_dvdnd_dat;

	/* For trivial cases */
	cmp_result=MPZ_abs_comp(m,a);
	if(cmp_result > 0 ){
		MPZ_copy(a,r);
		return 0;
	}else if(cmp_result == 0 ){
		MAC_MAKE_ZERO(*r);
		return 0;
	}

	/* normalization of the dividend and the dvider */
	normbits=(LONG_BITS - MPZ_nonzero_bits_num(m))%LONG_BITS;
	/*
		normbits=(LONG_BITS - MPZ_nonzero_bits_num(m))%LONG_BITS; 
	*/
	MPZ_shl(m,normbits,&dvder);
	MPZ_shl(a,normbits,&dvdnd);
	dvder1st = dvder.dat[dvder.len-1];
	dvder2nd = (dvder.len == 1)? 0 : dvder.dat[dvder.len-2];
	window.dat = &(dvdnd.dat[dvdnd.len - dvder.len]);
	window.len = dvder.len;
	loop = dvdnd.len - dvder.len;
	if((cmp_result=MPZ_abs_comp(&window,&dvder))>=0){
		MPZ_asym_sub(&window,&dvder,&window);
	}
	window.len=dvder.len+1;
	for(i = loop - 1  ; i >= 0 ; i--){
		window.dat--;
		window.len=dvder.len+1;
		if( window.dat[window.len-1] == dvder1st )
			tmp_quota = BITMASK_LONG;
		else
			//MAC_ULONG_DIVIDE1(&window.dat[window.len - 2], &window.dat[window.len - 1], &dvder1st, &tmp_quota);
			MAC_ULONG_DIVIDE(window.dat[window.len-2],window.dat[window.len-1],dvder1st,tmp_quota);
		while(1){
			x3=0; x2=0; /* must be initialized every time */
			MAC_ULONG_MULT(dvder2nd,tmp_quota,x1,x3);
			MAC_ULONG_MULT_ADD(dvder1st,tmp_quota,x2,x3);
			if(window.dat[window.len-1] > x3) break;
			if(window.dat[window.len-1] == x3 ){
				 if( window.dat[window.len-2] > x2 ) break;
				 if( window.dat[window.len-2] == x2 ){
					if( window.dat[window.len-3] >= x1 ) break;
				 }
			}
			tmp_quota--;
		}
		MPZ_mult_ULONG(&dvder,tmp_quota,&tmp_dvdnd);
		if(MPZ_abs_comp(&window,&tmp_dvdnd)>=0){
			MPZ_asym_sub(&window,&tmp_dvdnd,&window);
			if(window.dat[window.len-1]){
				MPZ_asym_sub(&window,&dvder,&window);
				tmp_quota++;
			}
		}else{
			/* Do NOT change the order of operations */
			/* following operations are asymmetric   */
			MPZ_asym_add(&window,&dvder,&window);
			MPZ_asym_sub(&window,&tmp_dvdnd,&window);
			tmp_quota--;
		}
	}

	if(normbits){
		MPZ_shr(&dvdnd,normbits,r);
	}else{
		MPZ_copy(&dvdnd,r);
	}
	r->sig=a->sig;

	MAC_CLR_UPPER_ZEROBYTES(*r);
	return 0;
}

SINT
MPZ_sub_mod(MPZ *a,MPZ *b,MPZ *m,MPZ *r)
{
	MPZ_sub(a,b,r);
	MPZ_mod(r,m,r);

	if(r->sig == -1)
		MPZ_add(m,r,r);

	return 0;
}

SINT
MPZ_mul_mod(MPZ *a,MPZ *b,MPZ *m,MPZ *r)
{
	MPZ_mul(a,b,r);
	MPZ_mod(r,m,r);

	return 0;
}

SINT
MPZ_mul_inv_mod(MPZ *a,MPZ *m,MPZ *b)
{
	MPZ ta, tb, x, y, q, r;
	MPZ *pta, *ptb, *px, *py, *pq, *pr, *pt;
	SINT sig;
	UINT ta_dat[MAX_MPZ_BUF_LEN], tb_dat[MAX_MPZ_BUF_LEN], x_dat[MAX_MPZ_BUF_LEN],
		y_dat[MAX_MPZ_BUF_LEN], q_dat[MAX_MPZ_BUF_LEN], r_dat[MAX_MPZ_BUF_LEN];
	int i=0;
	MAC_INIT_MPZ(ta,ta_dat,MPCONST_POS_SIG);
	MAC_INIT_MPZ(tb,tb_dat,MPCONST_POS_SIG);
	MAC_INIT_MPZ(x,x_dat,MPCONST_POS_SIG);
	MAC_INIT_MPZ(y,y_dat,MPCONST_POS_SIG);
	MAC_INIT_MPZ(q,q_dat,MPCONST_POS_SIG);
	MAC_INIT_MPZ(r,r_dat,MPCONST_POS_SIG);

	pta = &ta; ptb = &tb;
	px = &x; py = &y;
	pq = &q; pr = &r;

	/* step 1 */
	MAC_MAKE_ZERO(x); /* x = 0 */
	MAC_MAKE_ONE(y);  /* y = 1 */
	MPZ_copy(a,&ta);
	MPZ_copy(m,&tb);
	sig = 1;

	while(MAC_IS_POSITIVE_INTEGER(*ptb)){
		i++;
		/*step 2.1: pta = pq*ptb + pr */
		MPZ_div(pta,ptb,pq,pr); 
		pt = pta; /* This^ is mere a memory allocation to use pt temporarily for step 2.2*/
		pta = ptb;
		ptb = pr;

		/*step 2.2:	px = py - pq*px , py = px 
			since py and -px have same sign always MPZ_Add() used 
			inspite of MPZ_sub(). 	
		*/
		MPZ_mul(pq,px,pt);
		MPZ_add(py,pt,pt);
		pr = py; /* This is mere a memory allocation to use pr temporarily for step 2.1*/
		py = px;
		px = pt;
		sig = -sig;
	}

	/* step 3 */
	if(sig < 0)
		MPZ_sub(py,m,py);

	if(MAC_IS_MPZ_ONE(*pta))
		MPZ_mod(py,m,b);
	else
		MAC_MAKE_ZERO(*b);
	/* result is alway positive! */
	b->sig = MPCONST_POS_SIG;

	return 0;
}
