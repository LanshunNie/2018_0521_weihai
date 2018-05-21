package com.hit.heat.util;

public class GetPath {

	public static void main(String[] args) {
		// TODO Auto-generated method stub

	}

	 public static String getProjectPath() {
		 
	       java.net.URL url = GetPath.class .getProtectionDomain().getCodeSource().getLocation();
	       String filePath = null ;
	       try {
	           filePath = java.net.URLDecoder.decode (url.getPath(), "utf-8");
	       } catch (Exception e) {
	           e.printStackTrace();
	       }
	    if (filePath.endsWith(".jar"))
	       filePath = filePath.substring(0, filePath.lastIndexOf("/") + 1);
	    java.io.File file = new java.io.File(filePath);
	    filePath = file.getAbsolutePath();
	    return filePath;
	}
}
