package com.hit.heat.util;

public class ByteCrypt {

	private Byte key = 0B01011001;
	
	public ByteCrypt(){
		this(null);
	}
	public ByteCrypt(Byte key){
		if(key != null){
			this.key = key;
		}
	}
	
	/**
	 * 
	 * @param src
	 * @return
	 */
	public byte[] encrypt(String src){
		if(src == null){
			return null;
		}
		byte [] array = src.getBytes();
		for(int i = 0; i < array.length;i++){
			array[i] ^= key;
		}
		return array;
	}
	/**
	 * 
	 * @param src
	 * @return
	 */
	public byte[] encrypy(byte[] src){
		byte[] array = src.clone();
		for(int i=0;i< array.length;i++){
			array[i] ^= key;
		}
		return array;
	}
	
	/**
	 * 
	 * @param src
	 * @return
	 */
	public byte[] decrypt(byte[] src){
		byte [] array = src.clone();
		for(int i=0;i< array.length;i++){
			array[i] ^= key;
		}
		return array;
	}
}
