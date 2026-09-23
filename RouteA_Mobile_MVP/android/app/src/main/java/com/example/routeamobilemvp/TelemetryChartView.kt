package com.example.routeamobilemvp
import android.content.Context
import android.graphics.*
import android.view.View
class TelemetryChartView(c:Context):View(c){private val values=ArrayDeque<Float>();private val p=Paint(1).apply{color=Color.CYAN;strokeWidth=3f};fun add(v:Float){values.addLast(v);while(values.size>200)values.removeFirst();invalidate()};override fun onDraw(c:Canvas){super.onDraw(c);c.drawColor(Color.rgb(24,28,32));if(values.size<2)return;val a=values.toList();val path=Path();a.forEachIndexed{i,v->val x=i*width.toFloat()/(a.size-1);val y=height/2-v.coerceIn(-30f,30f)*height/70;if(i==0)path.moveTo(x,y)else path.lineTo(x,y)};c.drawPath(path,p)}}
