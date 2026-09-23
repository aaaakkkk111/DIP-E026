package com.example.routeamobilemvp
import android.content.Context
import android.security.keystore.KeyGenParameterSpec
import android.security.keystore.KeyProperties
import android.util.Base64
import java.security.KeyStore
import javax.crypto.Cipher
import javax.crypto.KeyGenerator
import javax.crypto.spec.GCMParameterSpec
class KeyVault(private val c:Context){private val alias="route_a_api";private val prefs=c.getSharedPreferences("secure",0)
    private fun key():java.security.Key{val ks=KeyStore.getInstance("AndroidKeyStore").apply{load(null)};ks.getKey(alias,null)?.let{return it};val g=KeyGenerator.getInstance(KeyProperties.KEY_ALGORITHM_AES,"AndroidKeyStore");g.init(KeyGenParameterSpec.Builder(alias,KeyProperties.PURPOSE_ENCRYPT or KeyProperties.PURPOSE_DECRYPT).setBlockModes(KeyProperties.BLOCK_MODE_GCM).setEncryptionPaddings(KeyProperties.ENCRYPTION_PADDING_NONE).build());return g.generateKey()}
    fun save(s:String){val x=Cipher.getInstance("AES/GCM/NoPadding");x.init(Cipher.ENCRYPT_MODE,key());prefs.edit().putString("v",Base64.encodeToString(x.iv+x.doFinal(s.toByteArray()),Base64.NO_WRAP)).apply()}
    fun load():String{val raw=prefs.getString("v",null)?:return "";return runCatching{val a=Base64.decode(raw,Base64.NO_WRAP);val x=Cipher.getInstance("AES/GCM/NoPadding");x.init(Cipher.DECRYPT_MODE,key(),GCMParameterSpec(128,a.copyOfRange(0,12)));String(x.doFinal(a.copyOfRange(12,a.size)))}.getOrDefault("")}
}
