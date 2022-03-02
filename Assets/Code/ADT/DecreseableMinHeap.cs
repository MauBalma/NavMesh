using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;

namespace Balma.ADT
{
	public class DecreseableMinHeapManaged<T>
	{
		List<T> _items;
		List<float> _values;
		Dictionary<T, int> _map;

		public DecreseableMinHeapManaged()
		{
			_items = new List<T>();
			_values = new List<float>();
			_map = new Dictionary<T, int>();
		}

		void Swap(int i, int j)
		{

			var tt = _items[i];
			_items[i] = _items[j];
			_items[j] = tt;

			var tv = _values[i];
			_values[i] = _values[j];
			_values[j] = tv;

			_map[_items[i]] = i;
			_map[_items[j]] = j;

		}

		public int Count => _items.Count;

		// Returns true if it found the item.
		bool DecreaseKey(T item, float newValue)
		{
			if (!_map.TryGetValue(item, out var current))
				return false;

			var oldValue = _values[current];
			if (oldValue > newValue)
			{
				_values[current] = newValue;

				// Bubble up
				while (current != 0)
				{
					var parent = (current - 1) / 2; //Integer division (round down)
					if (newValue < _values[parent])
					{
						Swap(current, parent);
						current = parent;
					}
					else
						break;
				}
			}

			return true;
		}

		public void Push(T item, float value)
		{
			// If the item already existed, update and exit
			if (DecreaseKey(item, value))
				return;

			// Add last (a leaf)
			_items.Add(item);
			_values.Add(value);

			var current = Count - 1;
			_map[item] = current;

			// Keep climbing to top until parent or unable to improve heap
			while (current != 0)
			{
				var parent = (current - 1) / 2; //Integer division (round down)
				if (value < _values[parent])
				{
					Swap(current, parent);
					current = parent;
				}
				else
					break;
			}
		}

		public T Peek()
		{
			return _items[0];
		}

		public T Pop(out float value)
		{
			var result = _items[0];
			value = _values[0];

			//Remove first, swap with last and decrease count
			_items[0] = _items[Count - 1];
			_values[0] = _values[Count - 1];
			_values.RemoveAt(Count - 1);
			_items.RemoveAt(Count - 1);

			if (Count > 0)
			{
				_map[_items[0]] = 0;

				// Trickle first down
				int current = 0;
				while (current < Count)
				{
					int child0 = current * 2 + 1;
					int child1 = current * 2 + 2;
					int best = current;
					if (child0 < Count && _values[child0] < _values[best])
						best = child0;
					if (child1 < Count && _values[child1] < _values[best])
						best = child1;

					if (best == current)
						break; // Can't improve
					else
					{
						Swap(current, best);
						current = best; // Go down that way
					}
				}
			}

			_map.Remove(result);
			return result;
		}

		public T Pop()
		{
			return Pop(out var v);
		}
	}
	
	public struct DecreseableMinHeap<T> : IDisposable where T : unmanaged, IEquatable<T>
	{
		NativeList<T> _items;
		NativeList<float> _values;
		NativeHashMap<T, int> _map;

		public DecreseableMinHeap(Allocator allocator)
		{
			_items = new NativeList<T>(allocator);
			_values = new NativeList<float>(allocator);
			_map = new NativeHashMap<T, int>(1024, allocator);
		}

		void Swap(int i, int j)
		{

			var tt = _items[i];
			_items[i] = _items[j];
			_items[j] = tt;

			var tv = _values[i];
			_values[i] = _values[j];
			_values[j] = tv;

			_map[_items[i]] = i;
			_map[_items[j]] = j;

		}

		public int Count => _items.Length;

		// Returns true if it found the item.
		bool DecreaseKey(T item, float newValue)
		{
			if (!_map.TryGetValue(item, out var current))
				return false;

			var oldValue = _values[current];
			if (oldValue > newValue)
			{
				_values[current] = newValue;

				// Bubble up
				while (current != 0)
				{
					var parent = (current - 1) / 2; //Integer division (round down)
					if (newValue < _values[parent])
					{
						Swap(current, parent);
						current = parent;
					}
					else
						break;
				}
			}

			return true;
		}

		public void Push(T item, float value)
		{
			// If the item already existed, update and exit
			if (DecreaseKey(item, value))
				return;

			// Add last (a leaf)
			_items.Add(item);
			_values.Add(value);

			var current = Count - 1;
			_map[item] = current;

			// Keep climbing to top until parent or unable to improve heap
			while (current != 0)
			{
				var parent = (current - 1) / 2; //Integer division (round down)
				if (value < _values[parent])
				{
					Swap(current, parent);
					current = parent;
				}
				else
					break;
			}
		}

		public T Peek()
		{
			return _items[0];
		}

		public T Pop(out float value)
		{
			var result = _items[0];
			value = _values[0];

			//Remove first, swap with last and decrease count
			_items[0] = _items[Count - 1];
			_values[0] = _values[Count - 1];
			_values.RemoveAt(Count - 1);
			_items.RemoveAt(Count - 1);

			if (Count > 0)
			{
				_map[_items[0]] = 0;

				// Trickle first down
				int current = 0;
				while (current < Count)
				{
					int child0 = current * 2 + 1;
					int child1 = current * 2 + 2;
					int best = current;
					if (child0 < Count && _values[child0] < _values[best])
						best = child0;
					if (child1 < Count && _values[child1] < _values[best])
						best = child1;

					if (best == current)
						break; // Can't improve
					else
					{
						Swap(current, best);
						current = best; // Go down that way
					}
				}
			}

			_map.Remove(result);
			return result;
		}

		public T Pop()
		{
			return Pop(out var v);
		}

		public void Dispose()
		{
			_items.Dispose();
			_values.Dispose();
			_map.Dispose();
		}

		public void Clear()
		{
			_items.Clear();
			_values.Clear();
			_map.Clear();
		}

		public DecreseableMinHeap<T> Copy(Allocator allocator)
		{
			var copy = new DecreseableMinHeap<T>();

			copy._items = new NativeList<T>(_items.Capacity, allocator);
			copy._items.CopyFrom(_items);
			
			copy._values = new NativeList<float>(_values.Capacity, allocator);
			copy._values.CopyFrom(_values);
			
			copy._map = new NativeHashMap<T, int>(_items.Capacity, allocator);
			var enumerator = _map.GetEnumerator();
			while (enumerator.MoveNext())
			{
				copy._map.Add(enumerator.Current.Key, enumerator.Current.Value);
			}

			return copy;
		}
	}
}