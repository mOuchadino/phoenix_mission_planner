package de.tum.stud.phoenix;

import java.util.HashMap;
import java.util.Map;

public class ApplicationContext {
	
	private static final Map<Class<?>, Object> nodes = new HashMap<Class<?>, Object>();
	
	@SuppressWarnings("unchecked")
	public synchronized static <T> T getNode(Class<T> clazz) {
		return (T) nodes.get(clazz);
	}
	
	public synchronized static <T> void put(Class<T> clazz, T node) {
		if (nodes.containsKey(clazz))
			System.out.println(String.format("Node for class %s will be replaced.", clazz));
		nodes.put(clazz, node);
	}
}