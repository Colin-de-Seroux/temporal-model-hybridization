AI

|_ graph<br>
|___ name<br>
|--- nodes<br>
|----- node<br>
|_______ name<br>
|------- components<br>
|--------- component<br>
|----------- type <subscriber | publisher | action | service><br>
|___________ name<br>
|----------- data <int | str | ...> (msg.type or return type)<br>
|----------- times <look after><br>
|----------- cpu_usage<br>
|----------- ram_usage<br>
|----------- placement<br>

Time :

- Time between publication and reception
- Time between launch and service execution
