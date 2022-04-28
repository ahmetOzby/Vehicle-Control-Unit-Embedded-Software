/*
 * AESK_comm_pro.c
 *
 *  Created on: Nov 28, 2020
 *      Author: basri
 */

#include "AESK_comm_pro.h"

/* AESK_comm_pro haberlesme hatlarinda kullanilmak uzere tasarlanmis bir yazilimsal kontrol katmanidir.
 * Bu paket yapisi sayesinde istek-cevap yonteminin kullanimi ve birbirinden farkli haberlesme
 * sistemlerinin tek bir cati altinda calistirilmasi amaclanmistir. Paket ;
 * 	uint8_t header_const_1;
	uint8_t header_const_2;
	uint8_t vehicle_id;
	uint8_t target_id;
	uint8_t source_id;
	uint8_t source_msg_id;
	uint8_t msg_size;
	uint8_t msg[MAX_MSG_SIZE];
	uint16_t msg_index;
	uint8_t CRC_L;
	uint8_t CRC_H;
 *	den olusmaktadir. Kullanici kolayligi saglamak amaciyla kutuphane parametre gir kullan yapisinda tasarlanmistir.
 *	Bunu calistiramayan da AESK'liyim demesin gardasim !!!! *
 *  Source-target yapisi kullanilmistir. Hicbir target kendisini ilgilendirmeyen paketi cozmeyecektir.
 */

/* Fonksiyon haberlesme ile ilgili temel degiskenleri tanimlamak icin kullanilir. İlk parametrede kanal adini verdigimiz paketinizi
 * cozecek struct'i girmek gerekli. Bu parametre sayesinde cozum islemi gerceklesen hafizada tutulmasi gereken tum degiskenler tutulacak.
 * struct icerisinde bulunan com_status struct'i haberlesme hattinizin saglik durumunu belirlemekte. Hat analizi de bu struct altinda yapilmaktadir.
 * 2.parametrede vehicle_id denen Aeske ait 3 aractan birinin idsini girmeniz gerekli. İlerleyen yillarda
 * arac tipi artabilir. Artarsa eger lutfen enuma ekleme yapin. Ek olarak; Bu parametre tamamen sov
 * 3.parametre yazilimin kostugu hardware'in idsi. Bu id sayesinde diger donanimlar verinin kimden geldigini ogrenecek. Su an icin 5 adet hardware mevcut.
 * İlerleyen yillarda kesinlikle artacaktir bu sayi. Lutfen enumu guncelleyin.
 * 4.parametre eger veriyi dogru gonderdiniz ve hat saglikli bi sekilde cozduyse odulunuzu alacaginiz function pointer. Gelen ham veriyi burada anlamlandiracaksiniz.
 * Lutfen function pointer bilmeden burada islem gerceklestirmeyin. Bir AESK'li function pointeri bilmeli!!!!!
   -yee
 */
void AESK_Compro_Init( 		Aesk_Compro_Settings* com_set,
					  const Vehicle_Id vehicle_id,
					  const Hardware_No source_id,
					  void(*pack_solver_func)(const Aesk_Compro_Pack*))
{
	for (uint16_t i = 0; i < sizeof(Aesk_Compro_Settings); i++)
	{
		((uint8_t*)com_set)[i] = 0;
	}
	com_set->vehicle_id_set = vehicle_id;
	com_set->source_id_set = source_id;
	com_set->pack_solver = pack_solver_func;
	com_set->state = H_CONST1_CNT;
}

/*
 * Kutsal AESK crc fonksiyonu. Sorgulamadan kullan gec. Yazanlara hayir duasi et.
 */
uint16_t AESK_calcCRC16(uint8_t *pt, size_t msgLen)
{
	uint16_t crc16_data = 0;
	uint8_t data = 0;

	for (uint16_t mlen = 0; mlen < msgLen; mlen++) {
		data = pt[mlen] ^ ((uint8_t)(crc16_data) & (uint8_t)(0xFF));
		data ^= data << 4;
		crc16_data = ((((uint16_t)data << 8) | ((crc16_data & 0xFF00) >> 8))
			^ (uint8_t)(data >> 4)
			^ ((uint16_t)data << 3));
	}
	return(crc16_data);
}

/*
 * Bu fonksiyon ile compro paketi olusturulmaktadir.
 * 1.parametrede olusturulacak olan compro paketinin struct'i girilecek. Bu struct paketi
 * olusturmak istediginiz blokta yerel degisken olarak cagirilabilir. Olusturulan paket ring_buffera gonderilecegi icin kullan - at yapilabilir. .
 * 2.parametrede vehicle_id girilecektir.
 * 3.parametrede mesajin iletilecegi target_id girilecektir. Burada 1'den fazla target olabilir. | islemi ile targetlari cogullayabilirsiniz.
 * Orn; hem telemetri hem de bms'e veri gondermek istediginizde bu parametreye TELEMETRI|BMS girebilirsiniz.
 * Girilen targetin oncelik sirasi yoktur.
 * 4.parametrede gonderilecek mesajin adresi girilir.
 * 5.parametrede gonderilecek mesajin boyutu girilmektedir. Gereksiz bayt yuklemesi yapmayalim lutfen. Gondereceginiz bayt kadarini girin.
 * Hatti yormayin bos yere.
 * 6.parametre gonderilecek mesajin idsini icermektedir. Mesajinizin hangi ID'de gonderilecegini yetkiliye sorun lutfen.
 */


void AESK_Compro_Create(	  Aesk_Compro_Pack* compro,
						const Vehicle_Id vehicle_id,
						const Hardware_No target_id,
						const Hardware_No source_id,
						const uint8_t* msg,
						const size_t msg_size,
						const uint8_t msg_id)
{
	uint16_t CRC_ = 0;
	static uint16_t indexa = 0;
	indexa++;
	compro->header_const_1 = HEADER_CONST1;
	compro->header_const_2 = HEADER_CONST2;
	compro->vehicle_id = vehicle_id;
	compro->target_id = target_id;
	compro->source_id = source_id;
	compro->source_msg_id = msg_id;
	compro->msg_size = msg_size;

	for(uint8_t i = 0; i < msg_size; i++)
	{
		compro->msg[i] = msg[i];
	}

	CRC_ = AESK_calcCRC16((uint8_t*)compro, msg_size + OFFSET);
	compro->msg_index_L = indexa;
	compro->msg_index_H = indexa >> 8;
	compro->CRC_L = (uint8_t)(CRC_ & 0x00FF);
	compro->CRC_H = (uint8_t)((CRC_ >> 8) & 0x00FF);

}

/*
 * Compro paketi olusturulduktan sonra ring_buffera gonderilmesi gerekmektedir. Compro paketi bu fonksiyon ile ring_buffera aktarilir.
 */
void AESK_Compro_Send_Ring_Buf(Aesk_Compro_Pack* compro, AESK_Ring_Buffer* ring_buf)
{
	uint8_t tmp_buf[255] = { 0 };
	uint8_t i = 0;
	for(; i < compro->msg_size + OFFSET; i++)
	{
		tmp_buf[i] = ((uint8_t*)compro)[i];
	}

	tmp_buf[i++] = compro->msg_index_L;
	tmp_buf[i++] = compro->msg_index_H;
	tmp_buf[i++] = compro->CRC_L;
	tmp_buf[i++] = compro->CRC_H;

	Write_Data_Ring_Buffer(ring_buf, tmp_buf, i);
}


/*
 * Yakalanan her bayt bu fonksiyonda degerlendirmeye alinir. Hattin sagligi, gelen bayt sayisi , paket indeks durumu, crc kontrolu
 * gibi islemler burada gerceklestirilir. Eger paket dogru gelmis ise cozme fonksiyonuna dallanir.
 */
void AESK_Compro_Unpack(Aesk_Compro_Settings* com_set,
						uint8_t* buf_rx,
						size_t size)
{
	com_set->com_status.received_byte += size; // gelen bayt sayisi surekli toplanir.
	for(uint64_t i = 0; i < size; i++) // gelen veri boyutu kadar cozme islemi gerceklestirilir.
	{
		switch (com_set->state)
		{
			case  H_CONST1_CNT : // Senkron olarak tanimlanan iki baytlik ozel degerlerden ilkinin gelip gelmedigi kontrol edilir.
			{
				if(buf_rx[i] == HEADER_CONST1)
				{
					com_set->aesk_compro.header_const_1 = buf_rx[i];
					com_set->state = H_CONST2_CNT; // geldiyse ikinciyi bekle
				}
				else
				{
					com_set->com_status.header_err++; //gelmediyse header error
					com_set->state = H_CONST1_CNT;
				}
				break;
			}
			case  H_CONST2_CNT : // Senkron olarak tanimlanan iki baytlik ozel degerlerden ikincisinin gelip gelmedigi kontrol edilir.
			{
				if(buf_rx[i] == HEADER_CONST2)
				{
					com_set->aesk_compro.header_const_2 = buf_rx[i];
					com_set->state = VEHIC_ID_CNT; // geldiyse vehicle_id kontrol et.
				}
				else
				{
					com_set->com_status.header_err++;  // gelmediyse header_error
					com_set->state = H_CONST1_CNT; //hata var basa don yeni paketi bekle
				}
				break;
			}

			case VEHIC_ID_CNT : // vehicle idnin dogru olup olmadigi kontrol edilir. TAMAMEN SOV
			{
				if(buf_rx[i] == com_set->vehicle_id_set)
				{
					com_set->aesk_compro.vehicle_id = com_set->vehicle_id_set;
					com_set->state = TARGET_ID_CNT; // dogruysa target kontrol
				}
				else
				{
					com_set->state = H_CONST1_CNT; // hata var basa don yeni paketi bekle
					com_set->com_status.vehicle_id_err++; // vehicle error artar
				}
				break;
			}
			case TARGET_ID_CNT : // Gelen mesajin target idsi secilen source_id yi iceriyor mu ? Kontrol edilir.
			{
				com_set->aesk_compro.target_id = buf_rx[i];
				com_set->state = SOURCE_ID_CNT;
				break;
			}

			case SOURCE_ID_CNT : // Cozme fonksiyonu icin gerekli. Hangi source'dan geldiyse ona gore cozme yapilacak.
			{
					com_set->aesk_compro.source_id = buf_rx[i];
					com_set->state = SOURCE_MSG_ID_CNT;// msg_idyi ogrenelim.
		
				break;
			}
			case SOURCE_MSG_ID_CNT : // MSG idyi ogrendik. Cozme islemi esnasinda bu da gerekli olacak.
			{
					com_set->aesk_compro.source_msg_id = buf_rx[i];
					com_set->state = MSG_SIZE_CNT; // kac baytlik mesaj gondermissiniz onu ogrenelim.
				
				break;
			}

			case MSG_SIZE_CNT : // MSG boyutunu ogreniyoruz.
			{
					com_set->aesk_compro.msg_size = buf_rx[i];
					com_set->state = MSG_CNT; // Mesaji toplayalim.
				break;
			}

			case MSG_CNT : // Mesaji toplayacagiz simdi. Gelen veri boyutuna gore burada biraz kalacagiz.
			{
				static uint8_t msg_size = 0;
				com_set->aesk_compro.msg[msg_size++] = buf_rx[i]; // msg_size kadar buffera ekleme yapiyoruz.
				if(msg_size >= com_set->aesk_compro.msg_size) // msg_size ulastiysak atlayalim.
				{
					com_set->state = MSG_INDEX_CNT_L; // crcyi ogrenelim.
					msg_size = 0; // msg_size sifirlaniyor. Sifirlanmazsa buralar alev alir alev. YANMAYALIM MI ERTAN??
				}
				break;
			}

			case MSG_INDEX_CNT_L : // indexin minik byteini ogreniyoruz.
			{
				com_set->current_index = buf_rx[i];
				com_set->state = MSG_INDEX_CNT_H; // abisini ogrenelim crcnin.
				break;
			}

			case MSG_INDEX_CNT_H : // indexin buyuk kardesi ogreniyoruz.
			{
				com_set->current_index = buf_rx[i] << 8 | com_set->current_index; // ilkel bir birlestirme islemi. Bu kismi anlamadiysan Necati Ergin c notunu bir oku.
				com_set->state = CRC_L_CNT; // mesajin indeksi aldik artik su crcyi bi cozelim.
				break;
			}

			case CRC_L_CNT : // crcnin minicik byteini ogreniyoruz.
			{
				com_set->aesk_compro.CRC_L= buf_rx[i];
				com_set->state = CRC_H_CNT;
				break;
			}

			case CRC_H_CNT : // crc'nin abisi gercekten crc miymis onu da ogrenecegiz birazdan...
			{
				uint16_t crc_calculated = 0;
				
				com_set->aesk_compro.CRC_H = buf_rx[i];
				
				crc_calculated = (com_set->aesk_compro.CRC_L)|((com_set->aesk_compro.CRC_H) << 8);// hala ogrenmediysen bu ilkel metodu birak kulubu
				
				// crc dogru mu degil mi cok heyecanli bakamiycam
				if( crc_calculated == AESK_calcCRC16((uint8_t*)&(com_set->aesk_compro), com_set->aesk_compro.msg_size + OFFSET))
				{
					//crc dogruymus simdi sira geldi indexe mesaj yeni bir mesaj mi yoksa daha once cozulmus bi mesaj mi geliyor bize
					//bu kisim birden fazla haberlesme sisteminin ortaklanmasi sonucunda ortaya cikan bir durumdur.
					if(index_control(com_set->current_index, &(com_set->current_index_matrix[com_set->aesk_compro.source_id])) == 1)
					{

						com_set->current_index_matrix[com_set->aesk_compro.source_id] = com_set->current_index;
						com_set->com_status.true_pack++; // gelen yeni bir mesajmis
						if(com_set->aesk_compro.target_id & com_set->source_id_set) //gelen mesaj bizeyse cozdurebiliriz demektir.
						{
							com_set->pack_solver(&com_set->aesk_compro); // hosgeldin reyiz gel seni bi guzel cozelim biz
						}
					}
					else
					{
						com_set->com_status.bad_err++; // eski paket bi ise yaramaz.
					}
				}

				else
				{
					com_set->com_status.CRC_err++; // patladik. ya hatta sikinti var ya da paketi yanlis olusturdun. gecmis olsun
				}
				com_set->state = H_CONST1_CNT; // basa donelim.
				break;
			}
			
			default : break;
		}
	}
}

/*
 * mesaja ait olan her yeni mesaj geldiginde 1 artan indeksin kontrolu bu fonksiyon ile gerceklestirilir.
 * 1.parametre paketle birlikte gelen indextir.
 * 2.parametre tamamen benim fantezim olan 2 boyutlu matrislerle belirlenmis olan kayit altinda tutulan indekslerdir.
 */
uint8_t index_control(uint16_t gelen_index, uint16_t *son_index)
{
	int32_t fark = 0;
	fark = gelen_index - *son_index;
	// fark 0 ile -10 arasindaysa bu paket daha once cozulmus demektir.
	//Deger 10'dan buyukse source reset yemis olarak kabul edilecektir.
	if(fark <= 0 && fark >= -10)
	{
		return 0;
	}

	else
	{
		*son_index = gelen_index; // gelen mesaj yeni bir mesajdir ve artik index yeni mesajin indexine aktarilmistir.
		return 1;
	}
}


