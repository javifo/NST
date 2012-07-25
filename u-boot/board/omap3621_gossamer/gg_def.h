/*
 * gg_def.h
 *
 *  Created on: Feb 3, 2011
 *      Author: x0151841
 */

#ifndef GG_DEF_H_
#define GG_DEF_H_

#define CHECK_ERROR_EXIT(cnd)		\
	do {				\
		if (cnd)		\
			goto exit;	\
	} while (0)

#define EXIT_WITH_ERROR exit:


#endif /* GG_DEF_H_ */
