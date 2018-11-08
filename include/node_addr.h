/*
 * node_addr.h
 *
 *  Created on: 02 марта 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef NODE_ADDR_H_
#define NODE_ADDR_H_

#ifndef CHAN_DEF
// Номер RF-канала по умолчанию
#define CHANN_DEF         0x00
#endif

#ifndef NODE_ADDR
// Собственный адрес нода по умолчанию
#define NODE_ADDR         0x02            // Собственный адрес нода по умолчанию
#endif

#define NET_ID            0x0101          // Идентификатор сети

//#define CHANN_DEF         ((NET_ID % 8)+1)   // RF-канал по умолчанию

#define BCRT_ADDR         0x00            // Адрес БКРТ-255
#define BRDCAST_ADDR      0xFF            // Широковещательный адрес


#endif /* NODE_ADDR_H_ */
