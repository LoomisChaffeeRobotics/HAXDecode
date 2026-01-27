package org.firstinspires.ftc.teamcode.practiceArchive;
import java.util.*;

public class lookUpTable<T> {
    private Map<T, List<Double>> table;

    // Constructor
    public lookUpTable() {
        this.table = new HashMap<>();
    }

    // Add an entry
    public void addEntry(T key, double num1, double num2, double num3) {
        List<Double> numbers = Arrays.asList(num1, num2, num3);
        table.put(key, numbers);
    }

    // Add an entry with list
    public void addEntry(T key, List<Double> numbers) {
        if (numbers.size() != 3) {
            throw new IllegalArgumentException("List must contain exactly 3 numbers");
        }
        table.put(key, new ArrayList<>(numbers));
    }

    // Get the list of 3 numbers for a key
    public List<Double> getNumbers(T key) {
        return table.get(key);
    }

    // Get numbers with default value if key not found
    public List<Double> getNumbers(T key, List<Double> defaultValue) {
        return table.getOrDefault(key, defaultValue);
    }

    // Check if key exists
    public boolean containsKey(T key) {
        return table.containsKey(key);
    }

    // Remove an entry
    public List<Double> removeEntry(T key) {
        return table.remove(key);
    }

    // Get all keys
    public Set<T> getAllKeys() {
        return table.keySet();
    }

    // Get all entries
    public Set<Map.Entry<T, List<Double>>> getAllEntries() {
        return table.entrySet();
    }

    // Get size of lookup table
    public int size() {
        return table.size();
    }

    // Clear all entries
    public void clear() {
        table.clear();
    }

    // Display the lookup table
    public void display() {
        System.out.println("Lookup Table Contents:");
        for (Map.Entry<T, List<Double>> entry : table.entrySet()) {
            System.out.println(entry.getKey() + " -> " + entry.getValue());
        }
    }

    // Convert to string representation
    @Override
    public String toString() {
        return table.toString();
    }
}