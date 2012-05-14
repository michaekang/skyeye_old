struct symbolInfo *symbol_info_head = NULL;

void add_list_tail(struct symbolInfo *list)
{
        static struct symbolInfo *symbol_info_tail = NULL;
        if(!symbol_info_head) {
                symbol_info_head = symbol_info_tail = list;
                return;
        }
        symbol_info_tail->next = list;
        symbol_info_tail = list;
}

uint8_t *store_string_info(char *info, size_t size)
{
        static uint32_t max_vol = 0x1000;
        static uint32_t offset = 0;
        static uint32_t remain = 0x1000;
        static uint8_t *repo = NULL;

        uint8_t *str = NULL;
        uint8_t *new_repo = NULL;
        struct symbolInfo *item = NULL;

        //printf("%s, %d, %d\n", info, size, remain);
        if (repo == NULL) {
                repo = (uint8_t *)malloc(max_vol);
                printf("allocate %d bytes.\n", max_vol);
        }
        if (remain < size) {
                new_repo = (uint8_t *)malloc(max_vol * 2);
                printf("allocate %d bytes.\n", max_vol * 2);
                memcpy(new_repo, repo, offset);
                for (item = symbol_info_head; item; item = item->next) {
                        //printf("symbol : %s\taddress : %x\n", item->name, item->address);
                        item->name = new_repo + ((uint8_t *)item->name - (uint8_t *)repo);
                }
                free(repo);
                repo = new_repo;
                new_repo = NULL;
                remain += max_vol;
                max_vol *= 2;
        }
        str = repo + offset;
        memcpy(repo + offset, info, size);
        repo[offset + size] = '\0';
        offset += size;
        remain -= size;
        return str;
}
struct symbolInfo *alloc_symbol_info(uint8_t *str, uint32_t address)
{
        struct symbolInfo *item = (struct symbolInfo *)malloc(sizeof(struct symbolInfo));
        if (item == NULL) {
                printf("Can't allocate more memory in %s\n", __FUNCTION__);
                exit(-1);
        }
        item->next = NULL;
        item->name = str;
        item->address = address;
        return item;
}

struct symbolInfo *search_symbol_info_by_addr(uint32_t address)
{
        struct symbolInfo *prev = NULL, *item = NULL;
        for (item = symbol_info_head; item; item = item->next) {
                if(address == item->address) {
                        return item;
                } else if(address > item->address){
                        prev = item;
                        continue;
                } else {
                        return prev;
                }
        }
        printf("Can not found the address 0x%x in System.map.\n", address);
        //exit(-1);
        return NULL;
}

void print_func_name(uint32_t address)
{
        static struct symbolInfo *last_found = NULL;
        static uint32_t last_address = 0;
        struct symbolInfo *new_found = NULL;
        new_found = search_symbol_info_by_addr(address);
        if (new_found == NULL) {
                return;
        }
        if (last_found != new_found) {
                if (last_found) {
                        LOG_IN_CLR(LIGHT_RED, "exit function %s 0x%x\n", last_found->name, last_address);
                }
                printf("%s\n", new_found->name);
                last_found = new_found;
                last_address = address;
        } else {
		last_address = address;
	}
}

void load_symbol_from_sysmap()
{
        char symbol_address[100];
        char symbol_name[100];
        char type = 0;
        char *str = NULL;
        struct symbolInfo *item = NULL;
        int i = 0;

        uint32_t address = 0;
        FILE *sysmap = fopen("/home/myesis/linux-2.6.35.y/System.map", "r");

        do {
                    if (3 != fscanf(sysmap, "%s %c %s", symbol_address, &type, symbol_name)) break;
                    address = strtol(symbol_address, NULL, 16);
                    while (symbol_name[i] != '\0') {
                            //printf("%c\n", symbol_name[i]);
                            i++;
                    }
                    //printf("symbol:%s\taddress:%x\tsize:%d\n", symbol_name, address, i);
                    str = (char *)store_string_info(symbol_name, i + 1);
                    item = alloc_symbol_info((uint8_t *)str, address);
                    add_list_tail(item);
        } while (1);
        for (item = symbol_info_head; item; item = item->next) {
                printf("symbol : %s\taddress : %x\n", item->name, item->address);
        }
}

